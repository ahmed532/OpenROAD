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
    case odb::dbChipRegion::Side::FRONT:
      return "FRONT";
    case odb::dbChipRegion::Side::BACK:
      return "BACK";
    case odb::dbChipRegion::Side::INTERNAL:
      return "INTERNAL";
    case odb::dbChipRegion::Side::INTERNAL_EXT:
      return "INTERNAL_EXT";
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
  // Use a local STA instance to avoid interference with the global OpenROAD
  // DB/STA state. We are parsing an external Verilog file that represents a
  // chiplet, which might not be part of the currently loaded top-level design
  // hierarchy in the way dbSta expects.
  // Initialize ONCE outside the loop for performance.
  // Note: This approach accumulates STA state for multiple chiplets.
  // For massive 3D systems with many large netlists, this could lead to high
  // memory usage. Since chiplets are often black-boxes/IO models, we prioritize
  // speed over clearing state between iterations.
  StaReportGuard local_sta;

  std::set<std::string> loaded_files;
  std::set<odb::dbChip*> processed_chips;

  for (auto chip_inst : chip->getChipInsts()) {
    odb::dbChip* master_prop = chip_inst->getMasterChip();

    if (!master_prop) {
      continue;
    }

    // We want the ChipletDef (master)
    if (processed_chips.contains(master_prop)) {
      continue;
    }
    processed_chips.insert(master_prop);

    // Check if property exists
    odb::dbProperty* prop = odb::dbProperty::find(master_prop, "verilog_file");
    if (!prop || prop->getType() != odb::dbProperty::STRING_PROP) {
      continue;
    }
    odb::dbStringProperty* sProp = static_cast<odb::dbStringProperty*>(prop);
    std::string verilog_file = sProp->getValue();

    // Read verilog if not already loaded
    if (!loaded_files.contains(verilog_file)) {
      logger_->info(
          utl::ODB,
          552,
          "Reading Verilog file {} for design {}",
          std::filesystem::path(verilog_file).filename().string().c_str(),
          master_prop->getName());
      if (!local_sta->readVerilog(verilog_file.c_str())) {
        logger_->warn(
            utl::ODB, 553, "Failed to read Verilog file {}", verilog_file);
        // If the file failed to load, we mark it as processed to suppress
        // identical errors for subsequent instances.
      }
      loaded_files.insert(verilog_file);
    }

    // We assume the design name matches the master name (Rule 1)
    std::string design_name = master_prop->getName();
    sta::Network* network = local_sta->network();
    sta::Cell* top_cell = nullptr;
    sta::LibraryIterator* lib_iter = network->libraryIterator();
    std::vector<std::string> available_modules;

    while (lib_iter->hasNext()) {
      sta::Library* lib = lib_iter->next();
      if (!lib) {  // paranoia check
        continue;
      }
      top_cell = network->findCell(lib, design_name.c_str());
      if (top_cell) {
        break;
      }
      // Gather available module names for diagnostics
      sta::PatternMatch all_cells("*");
      sta::CellSeq cells = network->findCellsMatching(lib, &all_cells);
      for (sta::Cell* cell : cells) {
        available_modules.emplace_back(network->name(cell));
      }
    }
    delete lib_iter;

    if (!top_cell) {
      odb::dbMarkerCategory* design_cat
          = odb::dbMarkerCategory::createOrGet(category, "Design Alignment");
      odb::dbMarker* marker = odb::dbMarker::create(design_cat);
      std::string modules_list
          = available_modules.empty()
                ? "None"
                : fmt::format("{}", fmt::join(available_modules, ", "));

      marker->setComment(fmt::format(
          "Rule 1 Violation: Verilog module {} not found in file {}. "
          "Available modules: {}",
          design_name,
          verilog_file,
          modules_list));
      marker->addSource(master_prop);

      logger_->warn(
          utl::ODB,
          550,
          "Rule 1 Violation: Failed to find Verilog module {} in file {}. "
          "Available modules: {}",
          design_name,
          verilog_file,
          modules_list);
      continue;
    }

    // Build a set of existing chip net names for O(1) lookup
    std::unordered_set<std::string> existing_nets;
    for (auto* ec_net : master_prop->getChipNets()) {
      existing_nets.insert(ec_net->getName());
    }

    // Iterate ports (terms) of the cell directly.
    sta::CellPortBitIterator* port_iter = network->portBitIterator(top_cell);
    while (port_iter->hasNext()) {
      sta::Port* port = port_iter->next();
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
  for (size_t i = 0; i < chips.size(); i++) {
    chip_to_idx[&chips[i]] = static_cast<int>(i);
  }

  for (const auto& conn : connections) {
    if (conn.isValid() && conn.top_region && conn.bottom_region) {
      auto it_top = chip_to_idx.find(conn.top_region->parent_chip);
      auto it_bot = chip_to_idx.find(conn.bottom_region->parent_chip);
      if (it_top != chip_to_idx.end() && it_bot != chip_to_idx.end()) {
        uf.unite(it_top->second, it_bot->second);
      }
    }
  }

  // 2. Union chips physically touching
  // Use a simple sort-on-X sweep to reduce O(N^2)
  std::vector<size_t> sorted_indices(chips.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::ranges::sort(sorted_indices, [&](size_t a, size_t b) {
    return chips[a].cuboid.xMin() < chips[b].cuboid.xMin();
  });

  for (size_t i = 0; i < sorted_indices.size(); i++) {
    size_t idx_i = sorted_indices[i];
    auto cuboid_i = chips[idx_i].cuboid;
    for (size_t j = i + 1; j < sorted_indices.size(); j++) {
      size_t idx_j = sorted_indices[j];
      if (chips[idx_j].cuboid.xMin() > cuboid_i.xMax()) {
        break;  // Cannot intersect any more chips in sorted order
      }
      if (cuboid_i.intersects(chips[idx_j].cuboid)) {
        uf.unite(static_cast<int>(idx_i), static_cast<int>(idx_j));
      }
    }
  }

  std::map<int, std::vector<const UnfoldedChip*>> sets;
  for (size_t i = 0; i < chips.size(); i++) {
    sets[uf.find(i)].push_back(&chips[i]);
  }

  if (sets.size() > 1) {
    std::vector<std::vector<const UnfoldedChip*>> insts_sets;
    insts_sets.reserve(sets.size());
    for (auto& [root, chips_list] : sets) {
      insts_sets.emplace_back(chips_list);
    }

    std::ranges::sort(insts_sets,
                      [](const std::vector<const UnfoldedChip*>& a,
                         const std::vector<const UnfoldedChip*>& b) {
                        return a.size() > b.size();
                      });

    odb::dbMarkerCategory* floating_chips_category
        = odb::dbMarkerCategory::createOrReplace(category, "Floating chips");
    int num_floating_sets = static_cast<int>(insts_sets.size()) - 1;
    logger_->warn(
        utl::ODB, 151, "Found {} floating chip sets", num_floating_sets);

    for (size_t i = 1; i < insts_sets.size(); i++) {
      auto& insts_set = insts_sets[i];
      odb::dbMarker* marker = odb::dbMarker::create(floating_chips_category);
      for (auto* inst : insts_set) {
        marker->addShape(Rect(inst->cuboid.xMin(),
                              inst->cuboid.yMin(),
                              inst->cuboid.xMax(),
                              inst->cuboid.yMax()));
        marker->addSource(inst->chip_inst_path.back());
      }
      marker->setComment("Isolated chip set starting with "
                         + insts_set[0]->getName());
    }
  }
}

void Checker::checkOverlappingChips(const UnfoldedModel& model,
                                    odb::dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  std::vector<std::pair<const UnfoldedChip*, const UnfoldedChip*>> overlaps;

  std::vector<size_t> sorted_indices(chips.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::ranges::sort(sorted_indices, [&](size_t a, size_t b) {
    return chips[a].cuboid.xMin() < chips[b].cuboid.xMin();
  });

  for (size_t i = 0; i < sorted_indices.size(); i++) {
    size_t idx_i = sorted_indices[i];
    auto cuboid_i = chips[idx_i].cuboid;
    for (size_t j = i + 1; j < sorted_indices.size(); j++) {
      size_t idx_j = sorted_indices[j];
      if (chips[idx_j].cuboid.xMin() >= cuboid_i.xMax()) {
        break;
      }
      auto cuboid_j = chips[idx_j].cuboid;
      if (chips[idx_i].isParentOf(&chips[idx_j])
          || chips[idx_j].isParentOf(&chips[idx_i])) {
        continue;
      }
      if (cuboid_i.overlaps(cuboid_j)) {
        auto intersection = cuboid_i.intersect(cuboid_j);
        if (!isOverlapFullyInConnections(
                model, &chips[idx_i], &chips[idx_j], intersection)) {
          overlaps.emplace_back(&chips[idx_i], &chips[idx_j]);
        }
      }
    }
  }

  if (!overlaps.empty()) {
    odb::dbMarkerCategory* overlapping_chips_category
        = odb::dbMarkerCategory::createOrReplace(category, "Overlapping chips");
    logger_->warn(
        utl::ODB, 156, "Found {} overlapping chips", (int) overlaps.size());

    for (const auto& [inst1, inst2] : overlaps) {
      odb::dbMarker* marker = odb::dbMarker::create(overlapping_chips_category);

      auto cuboid1 = inst1->cuboid;
      auto cuboid2 = inst2->cuboid;
      auto intersection = cuboid1.intersect(cuboid2);

      odb::Rect bbox(intersection.xMin(),
                     intersection.yMin(),
                     intersection.xMax(),
                     intersection.yMax());
      marker->addShape(bbox);

      marker->addSource(inst1->chip_inst_path.back());
      marker->addSource(inst2->chip_inst_path.back());

      std::string comment = "Chips " + inst1->getName() + " and "
                            + inst2->getName() + " overlap";
      marker->setComment(comment);
    }
  }
}

void Checker::checkConnectionRegions(const UnfoldedModel& model,
                                     dbChip* chip,
                                     dbMarkerCategory* category)
{
  const auto& connections = model.getConnections();
  odb::dbMarkerCategory* invalid_conn_category
      = odb::dbMarkerCategory::createOrReplace(category, "Connected regions");

  int non_intersecting_count = 0;

  for (const auto& conn : connections) {
    bool has_missing_region = (!conn.top_region || !conn.bottom_region)
                              && !conn.is_bterm_connection;

    // Skip marker and warning for intentional virtual connections
    // If it's virtual in 3dblox, it will have a nullptr in the dbChipConn for
    // one of the regions
    if (has_missing_region) {
      if (conn.connection->getTopRegion() == nullptr
          || conn.connection->getBottomRegion() == nullptr) {
        continue;
      }
      std::string top_status = conn.top_region ? "found" : "null";
      std::string bot_status = conn.bottom_region ? "found" : "null";
      logger_->warn(utl::ODB,
                    404,
                    "Connection {} has missing regions (top: {}, bottom: {})",
                    conn.connection->getName(),
                    top_status,
                    bot_status);
      continue;
    }

    if (!conn.isValid()) {
      odb::dbMarker* marker = odb::dbMarker::create(invalid_conn_category);
      marker->addSource(conn.connection);

      std::string top_info = "null";
      std::string bot_info = "null";

      if (conn.top_region) {
        marker->addSource(conn.top_region->region_inst);
        marker->addShape(Rect(conn.top_region->cuboid.xMin(),
                              conn.top_region->cuboid.yMin(),
                              conn.top_region->cuboid.xMax(),
                              conn.top_region->cuboid.yMax()));
        top_info = fmt::format(
            "{}/{} (faces {})",
            conn.top_region->parent_chip->getName(),
            conn.top_region->region_inst->getChipRegion()->getName(),
            sideToString(conn.top_region->effective_side));
      }
      if (conn.bottom_region) {
        marker->addSource(conn.bottom_region->region_inst);
        marker->addShape(Rect(conn.bottom_region->cuboid.xMin(),
                              conn.bottom_region->cuboid.yMin(),
                              conn.bottom_region->cuboid.xMax(),
                              conn.bottom_region->cuboid.yMax()));
        bot_info = fmt::format(
            "{}/{} (faces {})",
            conn.bottom_region->parent_chip->getName(),
            conn.bottom_region->region_inst->getChipRegion()->getName(),
            sideToString(conn.bottom_region->effective_side));
      }

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
  odb::dbMarkerCategory* unused_ext_category
      = odb::dbMarkerCategory::createOrReplace(category, "UnusedInternalExt");

  for (const auto& chip : model.getChips()) {
    for (const auto& region : chip.regions) {
      if (region.isInternalExt() && !region.isUsed) {
        logger_->warn(utl::ODB,
                      464,
                      "Region {} is INTERNAL_EXT but not included in any "
                      "Connectivity declaration.",
                      region.region_inst->getChipRegion()->getName());
        odb::dbMarker* marker = odb::dbMarker::create(unused_ext_category);
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
  // Rule 3: Physical alignment check
  // Verify that bumps are located within their parent region's footprint

  odb::dbMarkerCategory* alignment_category
      = odb::dbMarkerCategory::createOrReplace(category, "BumpAlignment");

  for (const auto& chip : model.getChips()) {
    for (const auto& region : chip.regions) {
      for (const auto& bump : region.bumps) {
        // Check if bump global XY is inside region global XY
        int x = bump.global_position.x();
        int y = bump.global_position.y();

        if (x < region.cuboid.xMin() || x > region.cuboid.xMax()
            || y < region.cuboid.yMin() || y > region.cuboid.yMax()) {
          odb::dbMarker* marker = odb::dbMarker::create(alignment_category);
          if (bump.bump_inst) {
            marker->addSource(bump.bump_inst);
          } else {
            marker->addSource(region.region_inst);  // Fallback
          }
          marker->addShape(Rect(x - kBumpMarkerHalfSize,
                                y - kBumpMarkerHalfSize,
                                x + kBumpMarkerHalfSize,
                                y + kBumpMarkerHalfSize));

          std::string msg = "Bump is outside its parent region "
                            + region.region_inst->getChipRegion()->getName();
          marker->setComment(msg);
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
  odb::dbMarkerCategory* open_net_category
      = odb::dbMarkerCategory::createOrReplace(category, "OpenNet");

  const auto& connections = model.getConnections();

  // For each net, check if all bumps are connected via valid physical paths.
  for (const auto& net : model.getNets()) {
    if (net.connected_bumps.size() < 2) {
      continue;
    }

    utl::UnionFind uf(net.connected_bumps.size());
    std::map<UnfoldedRegion*, std::vector<size_t>> bumps_by_region;
    for (size_t i = 0; i < net.connected_bumps.size(); i++) {
      bumps_by_region[net.connected_bumps[i]->parent_region].push_back(i);
    }

    std::map<UnfoldedChip*, std::vector<UnfoldedRegion*>> regions_by_chip;
    for (auto& [region, _] : bumps_by_region) {
      regions_by_chip[region->parent_chip].push_back(region);
    }

    for (auto& [chip, regions] : regions_by_chip) {
      size_t first_idx = bumps_by_region.at(regions[0])[0];
      for (auto* region : regions) {
        for (size_t idx : bumps_by_region.at(region)) {
          uf.unite((int) first_idx, (int) idx);
        }
      }
    }

    for (const auto& conn : connections) {
      if (!conn.isValid()) {
        continue;
      }
      auto it1 = bumps_by_region.find(conn.top_region);
      auto it2 = bumps_by_region.find(conn.bottom_region);

      if (it1 != bumps_by_region.end() && it2 != bumps_by_region.end()) {
        const auto& idxs1 = it1->second;
        const auto& idxs2 = it2->second;
        if (std::abs(net.connected_bumps[idxs1[0]]->global_position.z()
                     - net.connected_bumps[idxs2[0]]->global_position.z())
            <= conn.connection->getThickness()) {
          for (size_t i1 : idxs1) {
            for (size_t i2 : idxs2) {
              const auto& p1 = net.connected_bumps[i1]->global_position;
              const auto& p2 = net.connected_bumps[i2]->global_position;
              if (std::abs(p1.x() - p2.x()) <= bump_pitch_tolerance
                  && std::abs(p1.y() - p2.y()) <= bump_pitch_tolerance) {
                uf.unite((int) i1, (int) i2);
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
      auto max_group
          = std::max_element(groups.begin(), groups.end(), [](auto& a, auto& b) {
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
        odb::dbMarker* marker = odb::dbMarker::create(open_net_category);
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
  odb::dbMarkerCategory* alignment_category
      = odb::dbMarkerCategory::createOrGet(parent_category,
                                           "Logical Alignment");

  for (auto chip_inst : chip->getChipInsts()) {
    auto master = chip_inst->getMasterChip();
    if (processed_masters.contains(master)) {
      continue;
    }
    processed_masters.insert(master);

    // Check regions
    for (auto region : master->getChipRegions()) {
      for (auto bump : region->getChipBumps()) {
        odb::dbProperty* prop = odb::dbProperty::find(bump, "logical_net");
        if (!prop || prop->getType() != odb::dbProperty::STRING_PROP) {
          continue;
        }

        std::string logical_net
            = static_cast<odb::dbStringProperty*>(prop)->getValue();

        bool found = false;
        for (const auto& net : master->getChipNets()) {
          if (net->getName() == logical_net) {
            found = true;
            break;
          }
        }

        if (!found) {
          odb::dbMarker* marker = odb::dbMarker::create(alignment_category);
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
  for (const auto& conn : model.getConnections()) {
    if (!conn.isValid()) {
      continue;
    }

    UnfoldedRegion* r1 = conn.top_region;
    UnfoldedRegion* r2 = conn.bottom_region;

    if (!r1 || !r2) {
      continue;
    }

    bool match = (r1->parent_chip == chip1 && r2->parent_chip == chip2)
                 || (r1->parent_chip == chip2 && r2->parent_chip == chip1);

    if (match) {
      // If one of the regions is INTERNAL_EXT, it authorizes the overlap
      // of its parent chip with the other chip, but only within the region's
      // footprint.
      Rect r1_rect(r1->cuboid.xMin(),
                   r1->cuboid.yMin(),
                   r1->cuboid.xMax(),
                   r1->cuboid.yMax());
      Rect r2_rect(r2->cuboid.xMin(),
                   r2->cuboid.yMin(),
                   r2->cuboid.xMax(),
                   r2->cuboid.yMax());
      Rect overlap_rect(
          overlap.xMin(), overlap.yMin(), overlap.xMax(), overlap.yMax());

      if ((r1->isInternalExt() || r1->isInternal())
          && r1_rect.contains(overlap_rect)) {
        return true;
      }
      if ((r2->isInternalExt() || r2->isInternal())
          && r2_rect.contains(overlap_rect)) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace odb
