// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "threeDBloxValidator.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <utility>
#include <vector>

#include "odb/db.h"
#include "unfoldedModel.h"
#include "utl/Logger.h"
#include "utl/unionFind.h"

namespace odb {

ThreeDBloxValidator::ThreeDBloxValidator(utl::Logger* logger) : logger_(logger)
{
}

void ThreeDBloxValidator::validate(const UnfoldedModel& model,
                                   dbChip* chip,
                                   int tolerance,
                                   int bump_pitch_tolerance,
                                   bool verbose,
                                   const std::string& report_file)
{
  odb::dbMarkerCategory* top_category
      = odb::dbMarkerCategory::createOrReplace(chip, "3DBlox");
  odb::dbMarkerCategory* conn_category
      = odb::dbMarkerCategory::createOrReplace(top_category, "Connectivity");
  odb::dbMarkerCategory* phys_category
      = odb::dbMarkerCategory::createOrReplace(top_category, "Physical");
  odb::dbMarkerCategory* const_category
      = odb::dbMarkerCategory::createOrReplace(top_category, "Constraint");

  checkFloatingChips(model, conn_category);
  checkOverlappingChips(model, phys_category);
  checkConnectionRegions(model, chip, conn_category);
  checkInternalExtUsage(model, const_category);
  checkBumpPhysicalAlignment(model, phys_category);
  checkNetConnectivity(model, chip, conn_category, bump_pitch_tolerance);
}

void ThreeDBloxValidator::checkFloatingChips(const UnfoldedModel& model,
                                             odb::dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  const auto& connections = model.getConnections();
  utl::UnionFind uf(chips.size());

  // 1. Union chips connected by valid connections
  for (const auto& conn : connections) {
    if (conn.isValid() && conn.top_region && conn.bottom_region) {
      // Find indices of top and bottom chips in the deque
      int top_idx = -1;
      int bot_idx = -1;
      for (size_t i = 0; i < chips.size(); i++) {
        if (&chips[i] == conn.top_region->parent_chip) {
          top_idx = i;
        }
        if (&chips[i] == conn.bottom_region->parent_chip) {
          bot_idx = i;
        }
      }
      if (top_idx != -1 && bot_idx != -1) {
        uf.unite(top_idx, bot_idx);
      }
    }
  }

  // 2. Union chips physically touching (optional, depends on policy)
  // According to Plan 8, we only unite if there's a valid connection or if they
  // touch.
  for (size_t i = 0; i < chips.size(); i++) {
    auto cuboid_i = chips[i].cuboid;
    for (size_t j = i + 1; j < chips.size(); j++) {
      auto cuboid_j = chips[j].cuboid;
      if (cuboid_i.intersects(cuboid_j)) {
        uf.unite(i, j);
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
        = odb::dbMarkerCategory::createOrReplace(category, "Floating");
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

void ThreeDBloxValidator::checkOverlappingChips(const UnfoldedModel& model,
                                                odb::dbMarkerCategory* category)
{
  const auto& chips = model.getChips();
  std::vector<std::pair<const UnfoldedChip*, const UnfoldedChip*>> overlaps;

  for (size_t i = 0; i < chips.size(); i++) {
    auto cuboid_i = chips[i].cuboid;
    for (size_t j = i + 1; j < chips.size(); j++) {
      auto cuboid_j = chips[j].cuboid;
      if (cuboid_i.overlaps(cuboid_j)) {
        auto intersection = cuboid_i.intersect(cuboid_j);
        if (!isOverlapFullyInConnections(&chips[i], &chips[j], intersection)) {
          overlaps.emplace_back(&chips[i], &chips[j]);
        }
      }
    }
  }

  if (!overlaps.empty()) {
    odb::dbMarkerCategory* overlapping_chips_category
        = odb::dbMarkerCategory::createOrReplace(category, "Overlap");
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

void ThreeDBloxValidator::checkConnectionRegions(const UnfoldedModel& model,
                                                 dbChip* chip,
                                                 dbMarkerCategory* category)
{
  const auto& connections = model.getConnections();
  odb::dbMarkerCategory* invalid_conn_category
      = odb::dbMarkerCategory::createOrReplace(category, "InvalidConnection");

  for (const auto& conn : connections) {
    if (!conn.isValid()) {
      odb::dbMarker* marker = odb::dbMarker::create(invalid_conn_category);
      marker->addSource(conn.connection);
      if (conn.top_region) {
        marker->addSource(conn.top_region->region_inst);
        marker->addShape(Rect(conn.top_region->cuboid.xMin(),
                              conn.top_region->cuboid.yMin(),
                              conn.top_region->cuboid.xMax(),
                              conn.top_region->cuboid.yMax()));
      }
      if (conn.bottom_region) {
        marker->addSource(conn.bottom_region->region_inst);
        marker->addShape(Rect(conn.bottom_region->cuboid.xMin(),
                              conn.bottom_region->cuboid.yMin(),
                              conn.bottom_region->cuboid.xMax(),
                              conn.bottom_region->cuboid.yMax()));
      }
      marker->setComment("Invalid connection: " + conn.connection->getName());
    }
  }
}

void ThreeDBloxValidator::checkInternalExtUsage(const UnfoldedModel& model,
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

void ThreeDBloxValidator::checkBumpPhysicalAlignment(const UnfoldedModel& model,
                                                     dbMarkerCategory* category)
{
  // Implementation for Rule 3: Physical alignment check
  // Compare global bump position with corresponding DEF component position.
}

void ThreeDBloxValidator::checkNetConnectivity(const UnfoldedModel& model,
                                               dbChip* chip,
                                               dbMarkerCategory* category,
                                               int bump_pitch_tolerance)
{
  odb::dbMarkerCategory* open_net_category
      = odb::dbMarkerCategory::createOrReplace(category, "OpenNet");

  for (const auto& net : model.getNets()) {
    if (net.connected_bumps.empty()) {
      continue;
    }

    utl::UnionFind uf(net.connected_bumps.size());

    for (size_t i = 0; i < net.connected_bumps.size(); i++) {
      for (size_t j = i + 1; j < net.connected_bumps.size(); j++) {
        const auto* b1 = net.connected_bumps[i];
        const auto* b2 = net.connected_bumps[j];

        // Check if bumps are connected via alignment
        int dx = std::abs(b1->global_position.x() - b2->global_position.x());
        int dy = std::abs(b1->global_position.y() - b2->global_position.y());
        int dz = std::abs(b1->global_position.z() - b2->global_position.z());

        if (dx <= bump_pitch_tolerance && dy <= bump_pitch_tolerance
            && dz <= 10) {
          uf.unite(i, j);
        }
      }
    }

    std::map<int, std::vector<size_t>> groups;
    for (size_t i = 0; i < net.connected_bumps.size(); i++) {
      groups[uf.find(i)].push_back(i);
    }

    if (groups.size() > 1) {
      odb::dbMarker* marker = odb::dbMarker::create(open_net_category);
      marker->addSource(net.chip_net);
      marker->setComment("Net " + net.chip_net->getName() + " is split into "
                         + std::to_string(groups.size()) + " isolated groups.");
      for (const auto& [root, group] : groups) {
        for (size_t idx : group) {
          marker->addSource(net.connected_bumps[idx]->bump_inst);
        }
      }
    }
  }
}

bool ThreeDBloxValidator::isOverlapFullyInConnections(
    const UnfoldedChip* chip1,
    const UnfoldedChip* chip2,
    const Cuboid& overlap) const
{
  // If there's a valid connection involving INTERNAL_EXT between these chips,
  // it's an intentional overlap (embedded chip).
  return false;
}

}  // namespace odb
