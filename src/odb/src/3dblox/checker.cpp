// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "checker.h"

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ranges.h>

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "odb/db.h"
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

// RAII guard for local STA instance
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
    auto* log_cat  = dbMarkerCategory::createOrReplace(top_cat, "Logical");
  
    checkFloatingChips(model, conn_cat);
    checkOverlappingChips(model, phys_cat);
    checkConnectionRegions(model, chip, conn_cat);
    checkInternalExtUsage(model, conn_cat);
  
    checkBumpPhysicalAlignment(model, phys_cat);
    checkNetConnectivity(model, chip, conn_cat, bump_pitch_tolerance);
    checkConnectivity(chip, log_cat);
    checkLogicalAlignment(chip, log_cat);
  }
  

void Checker::checkConnectivity(dbChip* chip, dbMarkerCategory* category)
{
  StaReportGuard sta;
  std::unordered_set<std::string> loaded;
  std::unordered_set<dbChip*> processed;

  for (auto* inst : chip->getChipInsts()) {
    auto* master = inst->getMasterChip();
    if (!master || !processed.insert(master).second) {
      continue;
    }

    auto* prop = dbProperty::find(master, "verilog_file");
    if (!prop || prop->getType() != dbProperty::STRING_PROP) {
      continue;
    }

    std::string file = static_cast<dbStringProperty*>(prop)->getValue();
    if (loaded.insert(file).second) {
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
    std::string design = master->getName();
    sta::Cell* cell = nullptr;
    std::vector<std::string> modules;

    auto* lib_iter = network->libraryIterator();
    while (lib_iter->hasNext()) {
      auto* lib = lib_iter->next();
      if ((cell = network->findCell(lib, design.c_str()))) {
        break;
      }
      sta::PatternMatch all("*");
      for (auto* c : network->findCellsMatching(lib, &all)) {
        modules.emplace_back(network->name(c));
      }
    }
    delete lib_iter;

    if (!cell) {
      auto* cat = dbMarkerCategory::createOrGet(category, "Design Alignment");
      auto* marker = dbMarker::create(cat);
      std::string list = fmt::format("{}", fmt::join(modules, ", "));
      std::string msg = fmt::format(
          "Rule 1 Violation: Verilog module {} not found in file {}. Available "
          "modules: {}",
          design,
          file,
          modules.empty() ? "None" : list);
      marker->setComment(msg);
      marker->addSource(master);
      logger_->warn(utl::ODB, 550, msg);
      continue;
    }

    std::unordered_set<std::string_view> existing;
    for (auto* net : master->getChipNets()) {
      existing.insert(net->getName());
    }

    auto* port_iter = network->portBitIterator(cell);
    while (port_iter->hasNext()) {
      const char* name = network->name(port_iter->next());
      if (existing.insert(name).second) {
        dbChipNet::create(master, name);
        debugPrint(logger_,
                   utl::ODB,
                   "3dblox",
                   1,
                   "Created dbChipNet {} for chip {}",
                   name,
                   design);
      }
    }
    delete port_iter;
  }
}

void Checker::checkFloatingChips(const UnfoldedModel& model,
                                 dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  utl::UnionFind uf(chips.size());

  std::unordered_map<const UnfoldedChip*, int> chip_map;
  for (int i = 0; i < (int) chips.size(); ++i) {
    chip_map[&chips[i]] = i;
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

  for (int i = 0; i < (int) sorted.size(); ++i) {
    const auto& c1 = chips[sorted[i]].cuboid;
    for (int j = i + 1; j < (int) sorted.size(); ++j) {
      if (chips[sorted[j]].cuboid.xMin() > c1.xMax()) {
        break;
      }
      if (c1.intersects(chips[sorted[j]].cuboid)) {
        uf.unite(sorted[i], sorted[j]);
      }
    }
  }

  std::vector<std::vector<const UnfoldedChip*>> groups(chips.size());
  for (int i = 0; i < (int) chips.size(); ++i) {
    groups[uf.find(i)].push_back(&chips[i]);
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
  std::vector<std::pair<const UnfoldedChip*, const UnfoldedChip*>> overlaps;

  std::vector<int> sorted(chips.size());
  std::iota(sorted.begin(), sorted.end(), 0);
  std::ranges::sort(sorted, [&](int a, int b) {
    return chips[a].cuboid.xMin() < chips[b].cuboid.xMin();
  });

  for (int i = 0; i < (int) sorted.size(); ++i) {
    const auto& c1 = chips[sorted[i]].cuboid;
    for (int j = i + 1; j < (int) sorted.size(); ++j) {
      if (chips[sorted[j]].cuboid.xMin() >= c1.xMax()) {
        break;
      }
      if (chips[sorted[i]].isParentOf(&chips[sorted[j]])
          || chips[sorted[j]].isParentOf(&chips[sorted[i]])) {
        continue;
      }
      if (c1.overlaps(chips[sorted[j]].cuboid)) {
        auto intersect = c1.intersect(chips[sorted[j]].cuboid);
        if (!isOverlapFullyInConnections(
                model, &chips[sorted[i]], &chips[sorted[j]], intersect)) {
          overlaps.emplace_back(&chips[sorted[i]], &chips[sorted[j]]);
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
  for (const auto& net : model.getNets()) {
    if (net.connected_bumps.size() < 2) {
      continue;
    }

    utl::UnionFind uf(net.connected_bumps.size());
    std::unordered_map<UnfoldedChip*, int> chip_rep;
    std::unordered_map<UnfoldedRegion*, std::vector<int>> region_bumps;

    for (int i = 0; i < (int) net.connected_bumps.size(); ++i) {
      auto* b = net.connected_bumps[i];
      auto* chip = b->parent_region->parent_chip;
      if (!chip_rep.try_emplace(chip, i).second) {
        uf.unite(i, chip_rep[chip]);
      }
      region_bumps[b->parent_region].push_back(i);
    }

    for (const auto& conn : model.getConnections()) {
      if (!isValid(conn)) {
        continue;
      }
      auto it1 = region_bumps.find(conn.top_region);
      auto it2 = region_bumps.find(conn.bottom_region);
      if (it1 == region_bumps.end() || it2 == region_bumps.end()) {
        continue;
      }

      if (std::abs(net.connected_bumps[it1->second[0]]->global_position.z()
                   - net.connected_bumps[it2->second[0]]->global_position.z())
          <= conn.connection->getThickness()) {
        for (int i1 : it1->second) {
          for (int i2 : it2->second) {
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

    std::vector<std::vector<int>> groups(net.connected_bumps.size());
    for (int i = 0; i < (int) net.connected_bumps.size(); ++i) {
      groups[uf.find(i)].push_back(i);
    }
    std::erase_if(groups, [](const auto& g) { return g.empty(); });

    if (groups.size() > 1) {
      auto max_it = std::ranges::max_element(
          groups, [](auto& a, auto& b) { return a.size() < b.size(); });
      auto* marker = dbMarker::create(cat);
      marker->addSource(net.chip_net);
      int disconnected_count = 0;
      for (auto it = groups.begin(); it != groups.end(); ++it) {
        if (it == max_it) {
          continue;
        }
        for (int idx : *it) {
          disconnected_count++;
          if (net.connected_bumps[idx]->bump_inst) {
            marker->addSource(net.connected_bumps[idx]->bump_inst);
          }
        }
      }
      marker->setComment(
          fmt::format("Net {} has {} disconnected bump(s) out of {} total.",
                      net.chip_net->getName(),
                      disconnected_count,
                      net.connected_bumps.size()));
    }
  }
}

void Checker::checkLogicalAlignment(dbChip* chip,
                                    dbMarkerCategory* parent_category)
{
  std::unordered_set<dbChip*> processed;
  auto* cat
      = dbMarkerCategory::createOrGet(parent_category, "Logical Alignment");

  for (auto* inst : chip->getChipInsts()) {
    auto* master = inst->getMasterChip();
    if (!processed.insert(master).second) {
      continue;
    }

    std::unordered_set<std::string_view> net_names;
    for (auto* net : master->getChipNets()) {
      net_names.insert(net->getName());
    }

    for (auto* region : master->getChipRegions()) {
      for (auto* bump : region->getChipBumps()) {
        auto* net = bump->getNet();
        if (net && !net_names.contains(net->getName())) {
          auto* marker = dbMarker::create(cat);
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
    if ((r1->parent_chip == chip1 && r2->parent_chip == chip2)
        || (r1->parent_chip == chip2 && r2->parent_chip == chip1)) {
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

bool Checker::isValid(const UnfoldedConnection& conn) const
{
  auto* r1 = conn.top_region;
  auto* r2 = conn.bottom_region;
  if (!r1 || !r2) {
    return true;
  }
  if (!r1->cuboid.xyIntersects(r2->cuboid)) {
    return false;
  }
  if (r1->isInternalExt() || r2->isInternalExt()) {
    return true;
  }

  const auto& c1 = r1->cuboid;
  const auto& c2 = r2->cuboid;
  if ((r1->isInternal() || r2->isInternal())
      && std::max(c1.zMin(), c2.zMin()) <= std::min(c1.zMax(), c2.zMax())) {
    return true;
  }

  auto faces_up = [](auto* r) { return r->isFront() || r->isInternal(); };
  auto faces_down = [](auto* r) { return r->isBack() || r->isInternal(); };

  bool can_std = faces_down(r1) && faces_up(r2);
  bool can_inv = faces_up(r1) && faces_down(r2);
  if (!can_std && !can_inv) {
    return false;
  }

  bool r1_above = c1.zCenter() >= c2.zCenter();
  if (can_std && can_inv) {
    if (r1_above) {
      can_inv = false;
    } else {
      can_std = false;
    }
  }

  auto* top = can_std ? r1 : r2;
  auto* bot = can_std ? r2 : r1;
  int top_s = top->isInternal() ? top->cuboid.zMin() : top->getSurfaceZ();
  int bot_s = bot->isInternal() ? bot->cuboid.zMax() : bot->getSurfaceZ();

  return (top_s >= bot_s) && (top_s - bot_s <= conn.connection->getThickness());
}

}  // namespace odb
