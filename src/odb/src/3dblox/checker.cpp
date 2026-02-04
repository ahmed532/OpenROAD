// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "checker.h"

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "odb/db.h"
#include "odb/dbObject.h"
#include "odb/geom.h"
#include "sta/Network.hh"
#include "sta/PatternMatch.hh"
#include "sta/Sta.hh"
#include "unfoldedModel.h"
#include "utl/Logger.h"
#include "utl/unionFind.h"

namespace odb {

namespace {

constexpr int kBumpMarkerHalfSize = 50;

const char* sideToString(dbChipRegion::Side side)
{
  switch (side) {
    case dbChipRegion::Side::FRONT:
      return "FRONT";
    case dbChipRegion::Side::BACK:
      return "BACK";
    case dbChipRegion::Side::INTERNAL:
      return "INTERNAL";
    case dbChipRegion::Side::INTERNAL_EXT:
      return "INTERNAL_EXT";
  }
  return "UNKNOWN";
}

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
  sta::Sta* operator->() const { return sta_.get(); }

 private:
  std::unique_ptr<sta::Sta> sta_;
};

using ConnectionMap = std::unordered_map<
    const UnfoldedChip*,
    std::unordered_map<const UnfoldedChip*,
                       std::vector<const UnfoldedConnection*>>>;

}  // namespace

Checker::Checker(utl::Logger* logger) : logger_(logger)
{
}

void Checker::check(dbChip* chip, int bump_pitch_tolerance)
{
  UnfoldedModel model(logger_, chip);
  auto* top_cat = dbMarkerCategory::createOrReplace(chip, "3DBlox");
  auto* conn_cat = dbMarkerCategory::createOrReplace(top_cat, "Connectivity");
  auto* phys_cat = dbMarkerCategory::createOrReplace(top_cat, "Physical");
  auto* log_cat = dbMarkerCategory::createOrReplace(top_cat, "Logical");

  checkFloatingChips(model, conn_cat);
  checkOverlappingChips(model, phys_cat);
  checkConnectionRegions(model, chip, conn_cat);
  checkInternalExtUsage(model, conn_cat);

  checkBumpPhysicalAlignment(model, phys_cat);
  checkNetConnectivity(model, chip, conn_cat, bump_pitch_tolerance);

  checkLogical(chip, log_cat);
}

void Checker::checkLogical(dbChip* chip, dbMarkerCategory* category)
{
  StaReportGuard sta;
  std::unordered_set<std::string> loaded_files;
  std::unordered_set<dbChip*> processed_masters;

  auto* design_cat
      = dbMarkerCategory::createOrGet(category, "Design Alignment");
  auto* alignment_cat
      = dbMarkerCategory::createOrGet(category, "Logical Alignment");

  for (auto* inst : chip->getChipInsts()) {
    auto* master = inst->getMasterChip();
    if (!master || !processed_masters.insert(master).second) {
      continue;
    }

    auto* prop = dbProperty::find(master, "verilog_file");
    if (!prop || prop->getType() != dbProperty::STRING_PROP) {
      continue;
    }

    std::string file = static_cast<dbStringProperty*>(prop)->getValue();
    if (loaded_files.insert(file).second) {
      logger_->info(utl::ODB,
                    552,
                    "Reading Verilog file {} for design {}",
                    std::filesystem::path(file).filename().string(),
                    master->getName());
      if (!sta->readVerilog(file.c_str())) {
        logger_->warn(utl::ODB, 553, "Failed to read Verilog file {}", file);
      }
    }

    auto* network = sta->network();
    sta::Cell* cell = nullptr;
    auto* lib_iter = network->libraryIterator();
    while (lib_iter->hasNext()) {
      auto* lib = lib_iter->next();
      cell = network->findCell(lib, master->getName());
      if (cell) {
        break;
      }
    }
    delete lib_iter;

    if (!cell) {
      std::vector<std::string> modules;
      auto* li = network->libraryIterator();
      while (li->hasNext()) {
        sta::PatternMatch all("*");
        for (auto* c : network->findCellsMatching(li->next(), &all)) {
          modules.emplace_back(network->name(c));
        }
      }
      delete li;

      auto* marker = dbMarker::create(design_cat);
      std::string msg = fmt::format(
          "Rule 1 Violation: Verilog module {} not found in file {}. Available "
          "modules: {}",
          master->getName(),
          file,
          modules.empty() ? "None"
                          : fmt::format("{}", fmt::join(modules, ", ")));
      marker->setComment(msg);
      marker->addSource(master);
      logger_->warn(utl::ODB, 550, msg);
      continue;
    }

    std::unordered_set<std::string> verilog_nets;
    auto* port_iter = network->portBitIterator(cell);
    while (port_iter->hasNext()) {
      verilog_nets.insert(network->name(port_iter->next()));
    }
    delete port_iter;

    std::unordered_set<std::string> db_nets;
    for (auto* net : master->getChipNets()) {
      db_nets.insert(net->getName());
    }

    for (const auto& name : verilog_nets) {
      if (db_nets.insert(name).second) {
        dbChipNet::create(master, name);
        debugPrint(logger_,
                   utl::ODB,
                   "3dblox",
                   1,
                   "Created dbChipNet {} for chip {}",
                   name,
                   master->getName());
      }
    }

    for (auto* region : master->getChipRegions()) {
      for (auto* bump : region->getChipBumps()) {
        auto* net = bump->getNet();
        if (net && !verilog_nets.contains(net->getName())) {
          auto* marker = dbMarker::create(alignment_cat);
          marker->setComment(fmt::format("Logical net {} not found in Verilog",
                                         net->getName()));
          marker->addSource(bump);
          logger_->warn(
              utl::ODB,
              560,
              "Logical net {} in bmap for chiplet {} not found in Verilog",
              net->getName(),
              master->getName());
        }
      }
    }
  }
}

void Checker::checkFloatingChips(const UnfoldedModel& model,
                                 dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  utl::UnionFind uf(chips.size());
  std::unordered_map<const UnfoldedChip*, int> chip_map;
  for (size_t i = 0; i < chips.size(); ++i) {
    chip_map[&chips[i]] = (int) i;
  }

  for (const auto& conn : model.getConnections()) {
    if (isValid(conn) && conn.top_region && conn.bottom_region) {
      auto it1 = chip_map.find(conn.top_region->parent_chip);
      auto it2 = chip_map.find(conn.bottom_region->parent_chip);
      if (it1 != chip_map.end() && it2 != chip_map.end()) {
        uf.unite(it1->second, it2->second);
      }
    }
  }

  std::vector<int> sorted(chips.size());
  std::iota(sorted.begin(), sorted.end(), 0);
  std::ranges::sort(sorted, [&](int a, int b) {
    return chips[a].cuboid.xMin() < chips[b].cuboid.xMin();
  });

  for (size_t i = 0; i < sorted.size(); ++i) {
    const auto& c1 = chips[sorted[i]].cuboid;
    for (size_t j = i + 1; j < sorted.size(); ++j) {
      if (chips[sorted[j]].cuboid.xMin() > c1.xMax()) {
        break;
      }
      if (c1.intersects(chips[sorted[j]].cuboid)) {
        uf.unite(sorted[i], sorted[j]);
      }
    }
  }

  std::vector<std::vector<const UnfoldedChip*>> groups(chips.size());
  for (size_t i = 0; i < chips.size(); ++i) {
    groups[uf.find((int) i)].push_back(&chips[i]);
  }
  std::erase_if(groups, [](const auto& g) { return g.empty(); });

  if (groups.size() > 1) {
    std::ranges::sort(groups, [](const auto& a, const auto& b) {
      return a.size() > b.size();
    });
    auto* cat = dbMarkerCategory::createOrReplace(category, "Floating chips");
    logger_->warn(
        utl::ODB, 151, "Found {} floating chip sets", groups.size() - 1);
    for (size_t i = 1; i < groups.size(); ++i) {
      auto* marker = dbMarker::create(cat);
      for (auto* chip : groups[i]) {
        marker->addShape(Rect(chip->cuboid.xMin(),
                              chip->cuboid.yMin(),
                              chip->cuboid.xMax(),
                              chip->cuboid.yMax()));
        marker->addSource(chip->chip_inst_path.back());
      }
      marker->setComment("Isolated chip set starting with "
                         + groups[i][0]->name);
    }
  }
}

void Checker::checkOverlappingChips(const UnfoldedModel& model,
                                    dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  ConnectionMap conn_map;
  for (const auto& conn : model.getConnections()) {
    if (conn.top_region && conn.bottom_region) {
      auto* c1 = conn.top_region->parent_chip;
      auto* c2 = conn.bottom_region->parent_chip;
      conn_map[c1][c2].push_back(&conn);
      conn_map[c2][c1].push_back(&conn);
    }
  }

  std::vector<int> sorted(chips.size());
  std::iota(sorted.begin(), sorted.end(), 0);
  std::ranges::sort(sorted, [&](int a, int b) {
    return chips[a].cuboid.xMin() < chips[b].cuboid.xMin();
  });

  std::vector<std::pair<const UnfoldedChip*, const UnfoldedChip*>> overlaps;
  for (size_t i = 0; i < sorted.size(); ++i) {
    auto* c1 = &chips[sorted[i]];
    for (size_t j = i + 1; j < sorted.size(); ++j) {
      auto* c2 = &chips[sorted[j]];
      if (c2->cuboid.xMin() >= c1->cuboid.xMax()) {
        break;
      }
      if (c1->isParentOf(c2) || c2->isParentOf(c1)) {
        continue;
      }
      if (c1->cuboid.overlaps(c2->cuboid)) {
        if (!isOverlapFullyInConnections(
                model, c1, c2, c1->cuboid.intersect(c2->cuboid))) {
          overlaps.emplace_back(c1, c2);
        }
      }
    }
  }

  if (!overlaps.empty()) {
    auto* cat
        = dbMarkerCategory::createOrReplace(category, "Overlapping chips");
    logger_->warn(utl::ODB, 156, "Found {} overlapping chips", overlaps.size());
    for (const auto& [inst1, inst2] : overlaps) {
      auto* marker = dbMarker::create(cat);
      auto intersection = inst1->cuboid.intersect(inst2->cuboid);
      marker->addShape(Rect(intersection.xMin(),
                            intersection.yMin(),
                            intersection.xMax(),
                            intersection.yMax()));
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
  auto* cat = dbMarkerCategory::createOrReplace(category, "Connected regions");
  int count = 0;
  for (const auto& conn : model.getConnections()) {
    if (!conn.top_region || !conn.bottom_region) {
      if (conn.connection->getTopRegion()
          && conn.connection->getBottomRegion()) {
        logger_->warn(utl::ODB,
                      404,
                      "Connection {} has missing regions",
                      conn.connection->getName());
      }
      continue;
    }
    if (!isValid(conn)) {
      auto* marker = dbMarker::create(cat);
      marker->addSource(conn.connection);
      auto describe = [&](auto* r) {
        marker->addSource(r->region_inst);
        marker->addShape(Rect(r->cuboid.xMin(),
                              r->cuboid.yMin(),
                              r->cuboid.xMax(),
                              r->cuboid.yMax()));
        return fmt::format("{}/{} (faces {})",
                           r->parent_chip->name,
                           r->region_inst->getChipRegion()->getName(),
                           sideToString(r->effective_side));
      };
      std::string msg = fmt::format("Invalid connection {}: {} to {}",
                                    conn.connection->getName(),
                                    describe(conn.top_region),
                                    describe(conn.bottom_region));
      marker->setComment(msg);
      logger_->warn(utl::ODB, 207, msg);
      count++;
    }
  }
  if (count > 0) {
    logger_->warn(
        utl::ODB, 273, "Found {} non-intersecting connections", count);
  }
}

void Checker::checkInternalExtUsage(const UnfoldedModel& model,
                                    dbMarkerCategory* category)
{
  auto* cat = dbMarkerCategory::createOrReplace(category, "UnusedInternalExt");
  for (const auto& chip : model.getChips()) {
    for (const auto& region : chip.regions) {
      if (region.isInternalExt() && !region.isUsed) {
        logger_->warn(utl::ODB,
                      464,
                      "Region {} is INTERNAL_EXT but unused",
                      region.region_inst->getChipRegion()->getName());
        auto* marker = dbMarker::create(cat);
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
  auto* cat = dbMarkerCategory::createOrReplace(category, "BumpAlignment");
  for (const auto& chip : model.getChips()) {
    for (const auto& region : chip.regions) {
      for (const auto& bump : region.bumps) {
        const auto& p = bump.global_position;
        if (p.x() < region.cuboid.xMin() || p.x() > region.cuboid.xMax()
            || p.y() < region.cuboid.yMin() || p.y() > region.cuboid.yMax()) {
          auto* marker = dbMarker::create(cat);
          marker->addSource(bump.bump_inst ? (dbObject*) bump.bump_inst
                                           : (dbObject*) region.region_inst);
          marker->addShape(Rect(p.x() - kBumpMarkerHalfSize,
                                p.y() - kBumpMarkerHalfSize,
                                p.x() + kBumpMarkerHalfSize,
                                p.y() + kBumpMarkerHalfSize));
          marker->setComment(
              fmt::format("Bump is outside its parent region {}",
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
  auto* cat = dbMarkerCategory::createOrReplace(category, "OpenNet");
  int checked_nets = 0;

  for (const auto& net : model.getNets()) {
    if (net.connected_bumps.size() < 2) {
      continue;
    }
    checked_nets++;
    utl::UnionFind uf(net.connected_bumps.size());
    std::unordered_map<UnfoldedChip*, int> chip_rep;
    std::unordered_map<UnfoldedRegion*, std::vector<int>> region_bumps;

    for (size_t i = 0; i < net.connected_bumps.size(); ++i) {
      auto* b = net.connected_bumps[i];
      if (!chip_rep.try_emplace(b->parent_region->parent_chip, (int) i)
               .second) {
        uf.unite((int) i, chip_rep[b->parent_region->parent_chip]);
      }
      region_bumps[b->parent_region].push_back((int) i);
    }

    for (const auto& conn : model.getConnections()) {
      int top_z, bot_z;
      if (!getContactSurfaces(conn, top_z, bot_z)) {
        continue;
      }
      auto it1 = region_bumps.find(conn.top_region);
      auto it2 = region_bumps.find(conn.bottom_region);
      if (it1 == region_bumps.end() || it2 == region_bumps.end()) {
        continue;
      }

      // Surface distance check (Z)
      if (std::abs(top_z - bot_z) <= conn.connection->getThickness()) {
        std::unordered_map<int64_t, int> xy_map;
        auto pack = [](const Point3D& p) {
          return (static_cast<int64_t>(p.x()) << 32)
                 | (static_cast<uint32_t>(p.y()));
        };
        if (bump_pitch_tolerance == 0) {
          for (int i1 : it1->second) {
            xy_map[pack(net.connected_bumps[i1]->global_position)] = i1;
          }
          for (int i2 : it2->second) {
            auto hit
                = xy_map.find(pack(net.connected_bumps[i2]->global_position));
            if (hit != xy_map.end()) {
              uf.unite(hit->second, i2);
            }
          }
        } else {
          for (int i1 : it1->second) {
            const auto& p1 = net.connected_bumps[i1]->global_position;
            for (int i2 : it2->second) {
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

    std::vector<std::vector<int>> groups(net.connected_bumps.size());
    for (size_t i = 0; i < net.connected_bumps.size(); ++i) {
      groups[uf.find((int) i)].push_back((int) i);
    }
    std::erase_if(groups, [](const auto& g) { return g.empty(); });

    if (groups.size() > 1) {
      auto max_it = std::ranges::max_element(
          groups, [](auto& a, auto& b) { return a.size() < b.size(); });
      auto* marker = dbMarker::create(cat);
      marker->addSource(net.chip_net);
      int disconnected = 0;
      for (auto it = groups.begin(); it != groups.end(); ++it) {
        if (it == max_it) {
          continue;
        }
        for (int idx : *it) {
          disconnected++;
          if (net.connected_bumps[idx]->bump_inst) {
            marker->addSource(net.connected_bumps[idx]->bump_inst);
          }
        }
      }
      marker->setComment(
          fmt::format("Net {} has {} disconnected bump(s) out of {} total.",
                      net.chip_net->getName(),
                      disconnected,
                      net.connected_bumps.size()));
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
    if (!conn.top_region || !conn.bottom_region) {
      continue;
    }
    auto* r1 = conn.top_region;
    auto* r2 = conn.bottom_region;
    if ((r1->parent_chip == chip1 && r2->parent_chip == chip2)
        || (r1->parent_chip == chip2 && r2->parent_chip == chip1)) {
      if (!isValid(conn)) {
        continue;
      }
      auto auth = [&](auto* r) {
        return (r->isInternalExt() || r->isInternal())
               && Rect(r->cuboid.xMin(),
                       r->cuboid.yMin(),
                       r->cuboid.xMax(),
                       r->cuboid.yMax())
                      .contains(overlap_rect);
      };
      if (auth(r1) || auth(r2)) {
        return true;
      }
    }
  }
  return false;
}

bool Checker::getContactSurfaces(const UnfoldedConnection& conn,
                                 int& top_z,
                                 int& bot_z) const
{
  auto* r1 = conn.top_region;
  auto* r2 = conn.bottom_region;
  if (!r1 || !r2) {
    return false;
  }

  auto up = [](auto* r) { return r->isFront() || r->isInternal(); };
  auto down = [](auto* r) { return r->isBack() || r->isInternal(); };

  bool r1_down_r2_up = down(r1) && up(r2);
  bool r1_up_r2_down = up(r1) && down(r2);
  
  // std::cout << "[DEBUG] r1=" << r1->region_inst->getName() << " up=" << up(r1) << " down=" << down(r1) << std::endl;
  // std::cout << "[DEBUG] r2=" << r2->region_inst->getName() << " up=" << up(r2) << " down=" << down(r2) << std::endl;
  // std::cout << "[DEBUG] r1_down_r2_up=" << r1_down_r2_up << " r1_up_r2_down=" << r1_up_r2_down << std::endl;

  if (!r1_down_r2_up && !r1_up_r2_down) {
    // std::cout << "[DEBUG] returning false due to bad orientation" << std::endl;
    return false;
  }

  if (r1_down_r2_up && r1_up_r2_down) {
    if (r1->cuboid.zCenter() < r2->cuboid.zCenter()) {
      r1_down_r2_up = false;
    } else {
      r1_up_r2_down = false;
    }
  }

  auto* top = r1_down_r2_up ? r1 : r2;
  auto* bot = r1_down_r2_up ? r2 : r1;
  top_z = top->isInternal() ? top->cuboid.zMin() : top->getSurfaceZ();
  bot_z = bot->isInternal() ? bot->cuboid.zMax() : bot->getSurfaceZ();
  // std::cout << "[DEBUG] top=" << top->region_inst->getName() << " top_z=" << top_z << std::endl;
  // std::cout << "[DEBUG] bot=" << bot->region_inst->getName() << " bot_z=" << bot_z << std::endl;
  return true;
}

bool Checker::isValid(const UnfoldedConnection& conn) const
{
  int top_z, bot_z;
  if (!conn.top_region || !conn.bottom_region) {
    return true;
  }
  if (!conn.top_region->cuboid.xyIntersects(conn.bottom_region->cuboid)) {
    // std::cout << "[DEBUG] " << conn.connection->getName() << " xyIntersects failed" << std::endl;
    return false;
  }
  if (conn.top_region->isInternalExt() || conn.bottom_region->isInternalExt()) {
    return true;
  }

  if ((conn.top_region->isInternal() || conn.bottom_region->isInternal())
      && std::max(conn.top_region->cuboid.zMin(),
                  conn.bottom_region->cuboid.zMin())
             <= std::min(conn.top_region->cuboid.zMax(),
                         conn.bottom_region->cuboid.zMax())) {
    return true;
  }

  if (!getContactSurfaces(conn, top_z, bot_z)) {
    return false;
  }
  if (top_z < bot_z) {
    // std::cout << "[DEBUG] top_z < bot_z" << std::endl;
    return false;
  }
  return (top_z - bot_z) <= conn.connection->getThickness();
}

}  // namespace odb
