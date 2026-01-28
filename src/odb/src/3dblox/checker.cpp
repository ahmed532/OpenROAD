// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "checker.h"

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ranges.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "odb/db.h"
#include "odb/geom.h"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PatternMatch.hh"
#include "sta/Sta.hh"
#include "unfoldedModel.h"
#include "utl/Logger.h"
#include "utl/unionFind.h"

namespace odb {

// Marker size (in database units) for bump alignment violations.
// Uses a small fixed size since bumps are point-like features.
constexpr int kBumpMarkerHalfSize = 50;

static std::string sideToString(odb::dbChipRegion::Side side)
{
  switch (side) {
    case odb::dbChipRegion::Side::FRONT:        return "FRONT";
    case odb::dbChipRegion::Side::BACK:         return "BACK";
    case odb::dbChipRegion::Side::INTERNAL:     return "INTERNAL";
    case odb::dbChipRegion::Side::INTERNAL_EXT: return "INTERNAL_EXT";
  }
  return "UNKNOWN";
}

// RAII guard to prevent sta::Sta destructor from crashing by clearing
// ReportTcl.
class StaReportGuard
{
 public:
  StaReportGuard() : sta_(std::make_unique<sta::Sta>())
  {
    sta_->makeComponents();
  }
  ~StaReportGuard()
  {
    if (sta_) {
      sta_->setReport(nullptr);
    }
  }
  // Non-copyable, non-movable
  StaReportGuard(const StaReportGuard&) = delete;
  StaReportGuard& operator=(const StaReportGuard&) = delete;

  sta::Sta* operator->() const { return sta_.get(); }
  sta::Sta* get() const { return sta_.get(); }

 private:
  std::unique_ptr<sta::Sta> sta_;
};

Checker::Checker(utl::Logger* logger) : logger_(logger)
{
}

void Checker::check(odb::dbChip* chip, int bump_pitch_tolerance)
{
  UnfoldedModel model(logger_, chip);
  odb::dbMarkerCategory* top_category
      = odb::dbMarkerCategory::createOrReplace(chip, "3DBlox");

  odb::dbMarkerCategory* conn_category
      = odb::dbMarkerCategory::createOrReplace(top_category, "Connectivity");
  odb::dbMarkerCategory* phys_category
      = odb::dbMarkerCategory::createOrReplace(top_category, "Physical");
  odb::dbMarkerCategory* logical_category
      = odb::dbMarkerCategory::createOrReplace(top_category, "Logical");

  checkFloatingChips(model, conn_category);
  checkOverlappingChips(model, phys_category);
  checkConnectionRegions(model, chip, conn_category);
  checkInternalExtUsage(model, conn_category);

  checkBumpPhysicalAlignment(model, phys_category);
  checkNetConnectivity(model, chip, conn_category, bump_pitch_tolerance);
  checkConnectivity(chip, logical_category);
  checkLogicalAlignment(chip, logical_category);
}

void Checker::checkConnectivity(odb::dbChip* chip,
                                odb::dbMarkerCategory* category)
{
  StaReportGuard local_sta;
  std::set<std::string> loaded_files;
  std::set<odb::dbChip*> processed_chips;

  for (auto* chip_inst : chip->getChipInsts()) {
    auto* master_prop = chip_inst->getMasterChip();
    if (!master_prop || !processed_chips.insert(master_prop).second) {
      continue;
    }

    auto* prop = odb::dbProperty::find(master_prop, "verilog_file");
    if (!prop || prop->getType() != odb::dbProperty::STRING_PROP) {
      continue;
    }
    const std::string verilog_file
        = static_cast<odb::dbStringProperty*>(prop)->getValue();

    if (loaded_files.insert(verilog_file).second) {
      logger_->info(
          utl::ODB,
          552,
          "Reading Verilog file {} for design {}",
          std::filesystem::path(verilog_file).filename().string(),
          master_prop->getName());
      if (!local_sta->readVerilog(verilog_file.c_str())) {
        logger_->warn(
            utl::ODB, 553, "Failed to read Verilog file {}", verilog_file);
      }
    }

    const std::string design_name = master_prop->getName();
    auto* network = local_sta->network();
    sta::Cell* top_cell = nullptr;
    auto* lib_iter = network->libraryIterator();
    std::vector<std::string> available_modules;

    while (lib_iter->hasNext()) {
      auto* lib = lib_iter->next();
      if ((top_cell = network->findCell(lib, design_name.c_str()))) {
        break;
      }
      sta::PatternMatch all_cells("*");
      for (auto* cell : network->findCellsMatching(lib, &all_cells)) {
        available_modules.emplace_back(network->name(cell));
      }
    }
    delete lib_iter;

    if (!top_cell) {
      auto* design_cat
          = odb::dbMarkerCategory::createOrGet(category, "Design Alignment");
      auto* marker = odb::dbMarker::create(design_cat);
      std::string modules_list = fmt::format("{}", fmt::join(available_modules, ", "));

      std::string comment = fmt::format(
          "Rule 1 Violation: Verilog module {} not found in file {}. "
          "Available modules: {}",
          design_name,
          verilog_file,
          available_modules.empty() ? "None" : modules_list);
      marker->setComment(comment);
      marker->addSource(master_prop);

      logger_->warn(utl::ODB, 550, comment);
      continue;
    }

    std::unordered_set<std::string> existing_nets;
    for (auto* ec_net : master_prop->getChipNets()) {
      existing_nets.insert(ec_net->getName());
    }

    auto* port_iter = network->portBitIterator(top_cell);
    while (port_iter->hasNext()) {
      auto* port = port_iter->next();
      const char* port_name = network->name(port);

      if (!existing_nets.contains(port_name)) {
        odb::dbChipNet::create(master_prop, port_name);
        debugPrint(logger_,
                   utl::ODB,
                   "3dblox",
                   1,
                   "Created dbChipNet {} for chip {}",
                   port_name,
                   design_name);
      }
    }
    delete port_iter;
  }
}

void Checker::checkFloatingChips(const UnfoldedModel& model,
                                 odb::dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  const auto& connections = model.getConnections();
  utl::UnionFind uf(chips.size());

  // 1. Union chips connected by valid connections
  std::unordered_map<const UnfoldedChip*, int> chip_to_idx;
  for (int i = 0; i < (int) chips.size(); ++i) {
    chip_to_idx[&chips[i]] = i;
  }

  for (const auto& conn : connections) {
    if (isValid(conn) && conn.top_region && conn.bottom_region) {
      auto it_top = chip_to_idx.find(conn.top_region->parent_chip);
      auto it_bot = chip_to_idx.find(conn.bottom_region->parent_chip);
      if (it_top != chip_to_idx.end() && it_bot != chip_to_idx.end()) {
        uf.unite(it_top->second, it_bot->second);
      }
    }
  }

  // 2. Union chips physically touching
  std::vector<size_t> sorted_idx(chips.size());
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::ranges::sort(sorted_idx, [&](size_t a, size_t b) {
    return chips[a].cuboid.xMin() < chips[b].cuboid.xMin();
  });

  for (size_t i = 0; i < sorted_idx.size(); i++) {
    size_t idx_i = sorted_idx[i];
    auto cuboid_i = chips[idx_i].cuboid;
    for (size_t j = i + 1; j < sorted_idx.size(); j++) {
      size_t idx_j = sorted_idx[j];
      if (chips[idx_j].cuboid.xMin() > cuboid_i.xMax()) {
        break;
      }
      if (cuboid_i.intersects(chips[idx_j].cuboid)) {
        uf.unite((int) idx_i, (int) idx_j);
      }
    }
  }

  std::map<int, std::vector<const UnfoldedChip*>> groups;
  for (size_t i = 0; i < chips.size(); i++) {
    groups[uf.find((int) i)].push_back(&chips[i]);
  }

  if (groups.size() > 1) {
    std::vector<std::vector<const UnfoldedChip*>> sets;
    for (auto& [_, chip_list] : groups) {
      sets.push_back(std::move(chip_list));
    }

    std::ranges::sort(sets, [](const auto& a, const auto& b) {
      return a.size() > b.size();
    });

    auto* floating_cat
        = odb::dbMarkerCategory::createOrReplace(category, "Floating chips");
    logger_->warn(utl::ODB, 151, "Found {} floating chip sets", sets.size() - 1);

    for (size_t i = 1; i < sets.size(); i++) {
      auto* marker = odb::dbMarker::create(floating_cat);
      for (auto* inst : sets[i]) {
        marker->addShape(Rect(inst->cuboid.xMin(),
                              inst->cuboid.yMin(),
                              inst->cuboid.xMax(),
                              inst->cuboid.yMax()));
        marker->addSource(inst->chip_inst_path.back());
      }
      marker->setComment("Isolated chip set starting with " + sets[i][0]->name);
    }
  }
}

void Checker::checkOverlappingChips(const UnfoldedModel& model,
                                    odb::dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  std::vector<std::pair<const UnfoldedChip*, const UnfoldedChip*>> overlaps;

  std::vector<size_t> sorted_idx(chips.size());
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::ranges::sort(sorted_idx, [&](size_t a, size_t b) {
    return chips[a].cuboid.xMin() < chips[b].cuboid.xMin();
  });

  for (size_t i = 0; i < sorted_idx.size(); i++) {
    size_t idx_i = sorted_idx[i];
    auto cuboid_i = chips[idx_i].cuboid;
    for (size_t j = i + 1; j < sorted_idx.size(); j++) {
      size_t idx_j = sorted_idx[j];
      if (chips[idx_j].cuboid.xMin() >= cuboid_i.xMax()) {
        break;
      }
      auto cuboid_j = chips[idx_j].cuboid;
      if (chips[idx_i].isParentOf(&chips[idx_j])
          || chips[idx_j].isParentOf(&chips[idx_i])) {
        continue;
      }
      if (cuboid_i.overlaps(cuboid_j)) {
        auto intersect = cuboid_i.intersect(cuboid_j);
        if (!isOverlapFullyInConnections(
                model, &chips[idx_i], &chips[idx_j], intersect)) {
          overlaps.emplace_back(&chips[idx_i], &chips[idx_j]);
        }
      }
    }
  }

  if (!overlaps.empty()) {
    auto* overlap_cat
        = odb::dbMarkerCategory::createOrReplace(category, "Overlapping chips");
    logger_->warn(utl::ODB, 156, "Found {} overlapping chips", overlaps.size());

    for (const auto& [inst1, inst2] : overlaps) {
      auto* marker = odb::dbMarker::create(overlap_cat);
      auto intersect = inst1->cuboid.intersect(inst2->cuboid);

      marker->addShape(Rect(intersect.xMin(),
                            intersect.yMin(),
                            intersect.xMax(),
                            intersect.yMax()));

      marker->addSource(inst1->chip_inst_path.back());
      marker->addSource(inst2->chip_inst_path.back());

      marker->setComment(
          fmt::format("Chips {} and {} overlap", inst1->name, inst2->name));
    }
  }
}

void Checker::checkConnectionRegions(const UnfoldedModel& model,
                                     dbChip* chip,
                                     dbMarkerCategory* category)
{
  const auto& connections = model.getConnections();
  auto* invalid_conn_cat
      = odb::dbMarkerCategory::createOrReplace(category, "Connected regions");

  int non_intersecting_count = 0;

  for (const auto& conn : connections) {
    // Skip virtual connections (null regions are allowed)
    if (!conn.top_region || !conn.bottom_region) {
      if (conn.connection->getTopRegion() && conn.connection->getBottomRegion()) {
        logger_->warn(utl::ODB,
                      404,
                      "Connection {} has missing regions (top: {}, bottom: {})",
                      conn.connection->getName(),
                      conn.top_region ? "found" : "null",
                      conn.bottom_region ? "found" : "null");
      }
      continue;
    }

    if (!isValid(conn)) {
      auto* marker = odb::dbMarker::create(invalid_conn_cat);
      marker->addSource(conn.connection);

      auto process_region = [&](auto* region) {
        if (!region) {
          return std::string("null");
        }
        marker->addSource(region->region_inst);
        marker->addShape(Rect(region->cuboid.xMin(),
                              region->cuboid.yMin(),
                              region->cuboid.xMax(),
                              region->cuboid.yMax()));
        return fmt::format("{}/{} (faces {})",
                           region->parent_chip->name,
                           region->region_inst->getChipRegion()->getName(),
                           sideToString(region->effective_side));
      };

      std::string top_info = process_region(conn.top_region);
      std::string bot_info = process_region(conn.bottom_region);

      std::string comment = fmt::format("Invalid connection {}: {} to {}",
                                        conn.connection->getName(),
                                        top_info,
                                        bot_info);
      marker->setComment(comment);
      logger_->warn(utl::ODB, 207, comment);
      non_intersecting_count++;
    }
  }

  if (non_intersecting_count > 0) {
    logger_->warn(utl::ODB,
                  273,
                  "Found {} non-intersecting connections",
                  non_intersecting_count);
  }
}

void Checker::checkInternalExtUsage(const UnfoldedModel& model,
                                    dbMarkerCategory* category)
{
  auto* unused_ext_cat
      = odb::dbMarkerCategory::createOrReplace(category, "UnusedInternalExt");

  for (const auto& chip : model.getChips()) {
    for (const auto& region : chip.regions) {
      if (region.isInternalExt() && !region.isUsed) {
        logger_->warn(utl::ODB,
                      464,
                      "Region {} is INTERNAL_EXT but not included in any "
                      "Connectivity declaration.",
                      region.region_inst->getChipRegion()->getName());
        auto* marker = odb::dbMarker::create(unused_ext_cat);
        marker->addSource(region.region_inst);
        marker->addShape(Rect(region.cuboid.xMin(),
                              region.cuboid.yMin(),
                              region.cuboid.xMax(),
                              region.cuboid.yMax()));
        marker->setComment("Unused INTERNAL_EXT region: "
                           + region.region_inst->getChipRegion()->getName());
      }
    }
  }
}

void Checker::checkBumpPhysicalAlignment(const UnfoldedModel& model,
                                         dbMarkerCategory* category)
{
  auto* alignment_cat
      = odb::dbMarkerCategory::createOrReplace(category, "BumpAlignment");

  for (const auto& chip : model.getChips()) {
    for (const auto& region : chip.regions) {
      for (const auto& bump : region.bumps) {
        const auto& pos = bump.global_position;
        if (pos.x() < region.cuboid.xMin() || pos.x() > region.cuboid.xMax()
            || pos.y() < region.cuboid.yMin() || pos.y() > region.cuboid.yMax()) {
          auto* marker = odb::dbMarker::create(alignment_cat);
          marker->addSource(bump.bump_inst ? (dbObject*) bump.bump_inst
                                           : (dbObject*) region.region_inst);
          marker->addShape(Rect(pos.x() - kBumpMarkerHalfSize,
                                pos.y() - kBumpMarkerHalfSize,
                                pos.x() + kBumpMarkerHalfSize,
                                pos.y() + kBumpMarkerHalfSize));

          marker->setComment(fmt::format(
              "Bump is outside its parent region {}",
              region.region_inst->getChipRegion()->getName()));
        }
      }
    }
  }
}

void Checker::checkNetConnectivity(const UnfoldedModel& model,
                                   dbChip* chip,
                                   dbMarkerCategory* category,
                                   int bump_pitch_tolerance)
{
  auto* open_net_cat
      = odb::dbMarkerCategory::createOrReplace(category, "OpenNet");

  for (const auto& net : model.getNets()) {
    if (net.connected_bumps.size() < 2) {
      continue;
    }

    utl::UnionFind uf(net.connected_bumps.size());
    std::unordered_map<UnfoldedChip*, int> chip_rep;
    std::unordered_map<UnfoldedRegion*, std::vector<int>> bumps_by_region;

    for (int i = 0; i < (int) net.connected_bumps.size(); ++i) {
      auto* bump = net.connected_bumps[i];
      auto* chip = bump->parent_region->parent_chip;
      if (chip_rep.contains(chip)) {
        uf.unite(i, chip_rep[chip]);
      } else {
        chip_rep[chip] = i;
      }
      bumps_by_region[bump->parent_region].push_back(i);
    }

    for (const auto& conn : model.getConnections()) {
      if (!isValid(conn)) {
        continue;
      }
      auto it1 = bumps_by_region.find(conn.top_region);
      auto it2 = bumps_by_region.find(conn.bottom_region);

      if (it1 != bumps_by_region.end() && it2 != bumps_by_region.end()) {
        const auto& idxs1 = it1->second;
        const auto& idxs2 = it2->second;
        // Surface distance check (Z)
        if (std::abs(net.connected_bumps[idxs1[0]]->global_position.z()
                     - net.connected_bumps[idxs2[0]]->global_position.z())
            <= conn.connection->getThickness()) {
          for (int i1 : idxs1) {
            for (int i2 : idxs2) {
              const auto& p1 = net.connected_bumps[i1]->global_position;
              const auto& p2 = net.connected_bumps[i2]->global_position;
              if (std::abs(p1.x() - p2.x()) <= bump_pitch_tolerance
                  && std::abs(p1.y() - p2.y()) <= bump_pitch_tolerance) {
                uf.unite(i1, i2);
              }
            }
          }
        }
      }
    }

    std::map<int, std::vector<size_t>> groups;
    for (size_t i = 0; i < net.connected_bumps.size(); i++) {
      groups[uf.find((int) i)].push_back(i);
    }

    if (groups.size() > 1) {
      auto max_group = std::ranges::max_element(groups, [](auto& a, auto& b) {
        return a.second.size() < b.second.size();
      });

      std::vector<UnfoldedBump*> disconnected;
      for (auto& [root, indices] : groups) {
        if (root != max_group->first) {
          for (size_t idx : indices) {
            disconnected.push_back(net.connected_bumps[idx]);
          }
        }
      }

      if (!disconnected.empty()) {
        auto* marker = odb::dbMarker::create(open_net_cat);
        marker->addSource(net.chip_net);
        marker->setComment(
            fmt::format("Net {} has {} disconnected bump(s) out of {} total.",
                        net.chip_net->getName(),
                        disconnected.size(),
                        net.connected_bumps.size()));

        for (auto* bump : disconnected) {
          if (bump->bump_inst) {
            marker->addSource(bump->bump_inst);
          }
        }
      }
    }
  }
}

void Checker::checkLogicalAlignment(odb::dbChip* chip,
                                    odb::dbMarkerCategory* parent_category)
{
  std::set<odb::dbChip*> processed_masters;
  auto* alignment_cat = odb::dbMarkerCategory::createOrGet(parent_category,
                                                          "Logical Alignment");

  for (auto* chip_inst : chip->getChipInsts()) {
    auto* master = chip_inst->getMasterChip();
    if (!processed_masters.insert(master).second) {
      continue;
    }

    // Check regions
    for (auto* region : master->getChipRegions()) {
      for (auto* bump : region->getChipBumps()) {
        auto* net = bump->getNet();
        if (!net) {
          continue;
        }

        const std::string logical_net = net->getName();
        const auto nets = master->getChipNets();
        bool found = std::ranges::any_of(nets, [&](auto* net) {
          return net->getName() == logical_net;
        });

        if (!found) {
          auto* marker = odb::dbMarker::create(alignment_cat);
          std::string msg
              = "Logical net " + logical_net + " not found in Verilog";
          marker->setComment(msg);
          marker->addSource(bump);

          logger_->warn(utl::ODB,
                        560,
                        "Logical net {} in bmap for chiplet {} not found in "
                        "Verilog",
                        logical_net,
                        master->getName());
        }
      }
    }
  }
}

bool Checker::isOverlapFullyInConnections(const UnfoldedModel& model,
                                          const UnfoldedChip* chip1,
                                          const UnfoldedChip* chip2,
                                          const Cuboid& overlap) const
{
  if (chip1->isParentOf(chip2) || chip2->isParentOf(chip1)) {
    return true;
  }

  Rect overlap_rect(
      overlap.xMin(), overlap.yMin(), overlap.xMax(), overlap.yMax());

  for (const auto& conn : model.getConnections()) {
    if (!isValid(conn) || !conn.top_region || !conn.bottom_region) {
      continue;
    }

    auto* r1 = conn.top_region;
    auto* r2 = conn.bottom_region;

    bool match = (r1->parent_chip == chip1 && r2->parent_chip == chip2)
                 || (r1->parent_chip == chip2 && r2->parent_chip == chip1);

    if (match) {
      auto is_authorized = [&](auto* region) {
        if (!region->isInternalExt() && !region->isInternal()) {
          return false;
        }
        Rect r_rect(region->cuboid.xMin(),
                    region->cuboid.yMin(),
                    region->cuboid.xMax(),
                    region->cuboid.yMax());
        return r_rect.contains(overlap_rect);
      };

      if (is_authorized(r1) || is_authorized(r2)) {
        return true;
      }
    }
  }

  return false;
}

bool Checker::isValid(const UnfoldedConnection& conn) const
{
  if (!conn.top_region || !conn.bottom_region) {
    return true;
  }
  if (!conn.top_region->cuboid.xyIntersects(conn.bottom_region->cuboid)) {
    return false;
  }
  if (conn.top_region->isInternalExt() || conn.bottom_region->isInternalExt()) {
    return true;
  }

  const bool top_faces_down
      = conn.top_region->isBack() || conn.top_region->isInternal();
  const bool top_faces_up
      = conn.top_region->isFront() || conn.top_region->isInternal();
  const bool bot_faces_down
      = conn.bottom_region->isBack() || conn.bottom_region->isInternal();
  const bool bot_faces_up
      = conn.bottom_region->isFront() || conn.bottom_region->isInternal();

  bool standard_pair = top_faces_down && bot_faces_up;
  bool inverted_pair = top_faces_up && bot_faces_down;
  if (!standard_pair && !inverted_pair) {
    return false;
  }

  const Cuboid& t_cub = conn.top_region->cuboid;
  const Cuboid& b_cub = conn.bottom_region->cuboid;

  if ((conn.top_region->isInternal() || conn.bottom_region->isInternal())
      && std::max(t_cub.zMin(), b_cub.zMin())
             <= std::min(t_cub.zMax(), b_cub.zMax())) {
    return true;
  }

  int t_z = conn.top_region->getSurfaceZ();
  int b_z = conn.bottom_region->getSurfaceZ();

  if (standard_pair && inverted_pair && t_z < b_z) {
    std::swap(standard_pair, inverted_pair);
  }

  if (standard_pair) {
    if (conn.top_region->isInternal()) {
      t_z = t_cub.zMin();
    }
    if (conn.bottom_region->isInternal()) {
      b_z = b_cub.zMax();
    }
    if (t_z < b_z) {
      return false;
    }
  } else {
    if (conn.bottom_region->isInternal()) {
      b_z = b_cub.zMin();
    }
    if (conn.top_region->isInternal()) {
      t_z = t_cub.zMax();
    }
    if (b_z < t_z) {
      return false;
    }
  }

  return std::abs(t_z - b_z) <= conn.connection->getThickness();
}

}  // namespace odb
