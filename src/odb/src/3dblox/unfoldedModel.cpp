// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "unfoldedModel.h"

#include <algorithm>
#include <deque>
#include <map>
#include <ranges>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "odb/db.h"
#include "odb/dbTransform.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "utl/Logger.h"
#include "utl/unionFind.h"

namespace {

bool isPathZFlipped(const std::vector<odb::dbChipInst*>& path)
{
  bool flipped = false;
  for (auto inst : path) {
    if (inst->getOrient().isMirrorZ()) {
      flipped = !flipped;
    }
  }
  return flipped;
}

odb::dbChipRegion::Side computeEffectiveSide(
    odb::dbChipRegion::Side original,
    const std::vector<odb::dbChipInst*>& path)
{
  if (!isPathZFlipped(path)) {
    return original;
  }
  if (original == odb::dbChipRegion::Side::FRONT) {
    return odb::dbChipRegion::Side::BACK;
  }
  if (original == odb::dbChipRegion::Side::BACK) {
    return odb::dbChipRegion::Side::FRONT;
  }
  return original;
}

odb::dbTransform getTransform(odb::dbChipInst* inst)
{
  int z = inst->getLoc().z();
  if (inst->getOrient().isMirrorZ()) {
    z += inst->getMasterChip()->getThickness();
  }
  return odb::dbTransform(
      inst->getOrient(),
      odb::Point3D(inst->getLoc().x(), inst->getLoc().y(), z));
}

odb::Cuboid getCorrectedCuboid(odb::dbChipRegion* region)
{
  odb::dbChip* chip = region->getChip();
  odb::dbTech* tech = chip->getTech();
  if (!tech) {
    if (auto prop = odb::dbStringProperty::find(chip, "3dblox_tech")) {
      tech = chip->getDb()->findTech(prop->getValue().c_str());
    }
  }
  odb::dbTechLayer* layer = region->getLayer();
  if (!layer && tech) {
    if (auto prop = odb::dbStringProperty::find(region, "3dblox_layer")) {
      layer = tech->findLayer(prop->getValue().c_str());
    }
  }
  if (!layer || !tech) {
    return region->getCuboid();
  }

  uint32_t total = 0, layer_z = 0, target = 0;
  bool reached = false;
  for (auto l : tech->getLayers()) {
    uint32_t t = 0;
    if ((l->getType() == odb::dbTechLayerType::ROUTING
         || l->getType() == odb::dbTechLayerType::CUT)
        && l->getThickness(t)) {
      total += t;
      if (!reached) {
        layer_z += t;
      }
    }
    if (l == layer) {
      reached = true;
      layer->getThickness(target);
    }
  }
  int z_top
      = (region->getSide() == odb::dbChipRegion::Side::BACK)
            ? (int) layer_z
            : std::max(0, (int) chip->getThickness() - (int) (total - layer_z));
  odb::Rect box = region->getBox();
  return odb::Cuboid(box.xMin(),
                     box.yMin(),
                     z_top - (int) target,
                     box.xMax(),
                     box.yMax(),
                     z_top);
}

}  // namespace

namespace odb {

int UnfoldedRegion::getSurfaceZ() const
{
  if (isFront()) {
    return cuboid.zMax();
  }
  if (isBack()) {
    return cuboid.zMin();
  }
  return cuboid.zCenter();
}

bool UnfoldedConnection::isValid() const
{
  if (is_bterm_connection || !top_region || !bottom_region) {
    return true;
  }
  if (!top_region->cuboid.xyIntersects(bottom_region->cuboid)) {
    return false;
  }
  if (top_region->isInternalExt() || bottom_region->isInternalExt()) {
    return true;
  }

  auto faces = [](UnfoldedRegion* r) {
    return std::pair{r->isBack() || r->isInternal(),
                     r->isFront() || r->isInternal()};
  };
  auto [t_down, t_up] = faces(top_region);
  auto [b_down, b_up] = faces(bottom_region);

  bool standard_pair = t_down && b_up;
  bool inverted_pair = t_up && b_down;
  if (!standard_pair && !inverted_pair) {
    return false;
  }

  if ((top_region->isInternal() || bottom_region->isInternal())
      && std::max(top_region->cuboid.zMin(), bottom_region->cuboid.zMin())
             <= std::min(top_region->cuboid.zMax(),
                         bottom_region->cuboid.zMax())) {
    return true;
  }

  int t_z = top_region->getSurfaceZ();
  int b_z = bottom_region->getSurfaceZ();

  if (standard_pair && inverted_pair && t_z < b_z) {
    std::swap(standard_pair, inverted_pair);
  }

  if (standard_pair) {
    if (top_region->isInternal()) {
      t_z = top_region->cuboid.zMin();
    }
    if (bottom_region->isInternal()) {
      b_z = bottom_region->cuboid.zMax();
    }
    if (t_z < b_z) {
      return false;
    }
  } else {
    if (bottom_region->isInternal()) {
      b_z = bottom_region->cuboid.zMin();
    }
    if (top_region->isInternal()) {
      t_z = top_region->cuboid.zMax();
    }
    if (b_z < t_z) {
      return false;
    }
  }

  return std::abs(t_z - b_z) <= connection->getThickness();
}

bool UnfoldedChip::isParentOf(const UnfoldedChip* other) const
{
  if (chip_inst_path.size() >= other->chip_inst_path.size()) {
    return false;
  }
  return std::equal(chip_inst_path.begin(),
                    chip_inst_path.end(),
                    other->chip_inst_path.begin());
}

std::vector<UnfoldedBump*> UnfoldedNet::getDisconnectedBumps(
    utl::Logger* logger,
    const std::deque<UnfoldedConnection>& connections,
    int bump_pitch_tolerance) const
{
  if (connected_bumps.size() < 2) {
    return {};
  }

  utl::UnionFind uf(connected_bumps.size());
  std::map<UnfoldedRegion*, std::vector<size_t>> bumps_by_region;
  for (size_t i = 0; i < connected_bumps.size(); i++) {
    bumps_by_region[connected_bumps[i]->parent_region].push_back(i);
  }

  std::map<UnfoldedChip*, std::vector<UnfoldedRegion*>> regions_by_chip;
  for (auto& [region, _] : bumps_by_region) {
    regions_by_chip[region->parent_chip].push_back(region);
  }

  for (auto& [chip, regions] : regions_by_chip) {
    int first = -1;
    for (auto* region : regions) {
      for (size_t idx : bumps_by_region.at(region)) {
        if (first == -1) {
          first = (int) idx;
        } else {
          uf.unite(first, (int) idx);
        }
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
      if (std::abs(connected_bumps[idxs1[0]]->global_position.z()
                   - connected_bumps[idxs2[0]]->global_position.z())
          <= conn.connection->getThickness()) {
        for (size_t i1 : idxs1) {
          for (size_t i2 : idxs2) {
            if (std::abs(connected_bumps[i1]->global_position.x()
                         - connected_bumps[i2]->global_position.x())
                    <= bump_pitch_tolerance
                && std::abs(connected_bumps[i1]->global_position.y()
                            - connected_bumps[i2]->global_position.y())
                       <= bump_pitch_tolerance) {
              uf.unite((int) i1, (int) i2);
            }
          }
        }
      }
    }
  }

  std::map<int, std::vector<size_t>> groups;
  for (size_t i = 0; i < connected_bumps.size(); i++) {
    groups[uf.find((int) i)].push_back(i);
  }
  if (groups.size() <= 1) {
    return {};
  }

  auto max_group
      = std::max_element(groups.begin(), groups.end(), [](auto& a, auto& b) {
          return a.second.size() < b.second.size();
        });

  std::vector<UnfoldedBump*> disconnected;
  for (auto& [root, indices] : groups) {
    if (root != max_group->first) {
      for (size_t idx : indices) {
        disconnected.push_back(connected_bumps[idx]);
      }
    }
  }
  return disconnected;
}

std::string UnfoldedChip::getName() const
{
  std::string name;
  for (auto* inst : chip_inst_path) {
    if (!name.empty()) {
      name += '/';
    }
    name += inst->getName();
  }
  return name;
}

UnfoldedModel::UnfoldedModel(utl::Logger* logger, dbChip* chip)
    : logger_(logger)
{
  for (dbChipInst* inst : chip->getChipInsts()) {
    std::vector<dbChipInst*> path;
    Cuboid local;
    buildUnfoldedChip(inst, path, local);
  }
  unfoldConnectionsRecursive(chip, {});
  unfoldNetsRecursive(chip);
}

UnfoldedChip* UnfoldedModel::buildUnfoldedChip(dbChipInst* inst,
                                               std::vector<dbChipInst*>& path,
                                               Cuboid& local)
{
  dbChip* master = inst->getMasterChip();
  path.push_back(inst);
  UnfoldedChip uf_chip{.chip_inst_path = path};

  if (master->getChipType() == dbChip::ChipType::HIER) {
    uf_chip.cuboid.mergeInit();
    for (auto sub : master->getChipInsts()) {
      Cuboid sub_local;
      buildUnfoldedChip(sub, path, sub_local);
      uf_chip.cuboid.merge(sub_local);
    }
  } else {
    uf_chip.cuboid = master->getCuboid();
  }

  local = uf_chip.cuboid;
  getTransform(inst).apply(local);

  dbTransform total;
  for (auto i : path | std::views::reverse) {
    total.concat(getTransform(i), total);
  }
  uf_chip.transform = total;
  total.apply(uf_chip.cuboid);
  uf_chip.z_flipped = isPathZFlipped(path);

  for (auto* region_inst : inst->getRegions()) {
    UnfoldedRegion uf_region{
        .region_inst = region_inst,
        .effective_side
        = computeEffectiveSide(region_inst->getChipRegion()->getSide(), path),
        .cuboid = getCorrectedCuboid(region_inst->getChipRegion())};
    total.apply(uf_region.cuboid);
    unfoldBumps(uf_region, total);
    uf_chip.regions.push_back(uf_region);
  }

  unfolded_chips_.push_back(uf_chip);
  UnfoldedChip* ptr = &unfolded_chips_.back();
  for (auto& region : ptr->regions) {
    region.parent_chip = ptr;
    ptr->region_map[region.region_inst] = &region;
    for (auto& bump : region.bumps) {
      bump.parent_region = &region;
      bump_inst_map_[bump.bump_inst] = &bump;
    }
  }
  chip_path_map_[ptr->chip_inst_path] = ptr;
  path.pop_back();
  return ptr;
}

void UnfoldedModel::unfoldBumps(UnfoldedRegion& uf_region,
                                const dbTransform& transform)
{
  for (auto* bump_inst : uf_region.region_inst->getChipBumpInsts()) {
    dbChipBump* bump = bump_inst->getChipBump();
    if (!bump->getInst()) {
      continue;
    }
    Point global_xy = bump->getInst()->getLocation();
    transform.apply(global_xy);
    uf_region.bumps.push_back(
        {.bump_inst = bump_inst,
         .parent_region = &uf_region,
         .global_position
         = Point3D(global_xy.x(), global_xy.y(), uf_region.getSurfaceZ())});
  }
}

UnfoldedChip* UnfoldedModel::findUnfoldedChip(
    const std::vector<dbChipInst*>& path)
{
  auto it = chip_path_map_.find(path);
  return it != chip_path_map_.end() ? it->second : nullptr;
}

UnfoldedRegion* UnfoldedModel::findUnfoldedRegion(UnfoldedChip* chip,
                                                  dbChipRegionInst* inst)
{
  if (!chip || !inst) {
    return nullptr;
  }
  auto it = chip->region_map.find(inst);
  return it != chip->region_map.end() ? it->second : nullptr;
}

void UnfoldedModel::unfoldConnectionsRecursive(
    dbChip* chip,
    const std::vector<dbChipInst*>& parent_path)
{
  for (auto* conn : chip->getChipConns()) {
    auto full_path = [&](const std::vector<dbChipInst*>& rel) {
      auto full = parent_path;
      full.insert(full.end(), rel.begin(), rel.end());
      return full;
    };

    UnfoldedRegion* top = findUnfoldedRegion(
        findUnfoldedChip(full_path(conn->getTopRegionPath())),
        conn->getTopRegion());
    UnfoldedRegion* bot = findUnfoldedRegion(
        findUnfoldedChip(full_path(conn->getBottomRegionPath())),
        conn->getBottomRegion());

    if (top || bot) {
      UnfoldedConnection uf_conn{
          .connection = conn, .top_region = top, .bottom_region = bot};
      if (top && top->isInternalExt()) {
        top->isUsed = true;
      }
      if (bot && bot->isInternalExt()) {
        bot->isUsed = true;
      }
      unfolded_connections_.push_back(uf_conn);
    }
  }

  for (auto* inst : chip->getChipInsts()) {
    auto sub = parent_path;
    sub.push_back(inst);
    unfoldConnectionsRecursive(inst->getMasterChip(), sub);
  }
}

void UnfoldedModel::unfoldNetsRecursive(dbChip* chip)
{
  for (auto* net : chip->getChipNets()) {
    UnfoldedNet uf_net{.chip_net = net};
    for (uint32_t i = 0; i < net->getNumBumpInsts(); i++) {
      std::vector<dbChipInst*> path;
      auto it = bump_inst_map_.find(net->getBumpInst(i, path));
      if (it != bump_inst_map_.end()) {
        uf_net.connected_bumps.push_back(it->second);
      }
    }
    unfolded_nets_.push_back(uf_net);
  }
  for (auto* inst : chip->getChipInsts()) {
    unfoldNetsRecursive(inst->getMasterChip());
  }
}

}  // namespace odb