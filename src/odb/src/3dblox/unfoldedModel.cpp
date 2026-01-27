// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "unfoldedModel.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

std::string getChipPathKey(const std::vector<odb::dbChipInst*>& path)
{
  std::string key;
  for (auto* inst : path) {
    if (!key.empty()) {
      key += '/';
    }
    key += inst->getName();
  }
  return key;
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

odb::Cuboid computeConnectionCuboid(const odb::UnfoldedRegion& top,
                                    const odb::UnfoldedRegion& bottom)
{
  odb::Cuboid result;
  result.set_xlo(std::max(top.cuboid.xMin(), bottom.cuboid.xMin()));
  result.set_ylo(std::max(top.cuboid.yMin(), bottom.cuboid.yMin()));
  result.set_xhi(std::min(top.cuboid.xMax(), bottom.cuboid.xMax()));
  result.set_yhi(std::min(top.cuboid.yMax(), bottom.cuboid.yMax()));
  result.set_zlo(std::min(top.getSurfaceZ(), bottom.getSurfaceZ()));
  result.set_zhi(std::max(top.getSurfaceZ(), bottom.getSurfaceZ()));
  return result;
}

odb::dbTransform getTransform(odb::dbChipInst* inst)
{
  int z = inst->getLoc().z();
  if (inst->getOrient().isMirrorZ()) {
    z += inst->getMasterChip()->getThickness();
  }
  return odb::dbTransform(inst->getOrient(),
                          odb::Point3D(inst->getLoc().x(), inst->getLoc().y(), z));
}

odb::Cuboid getCorrectedCuboid(odb::dbChipRegion* region)
{
  odb::dbTechLayer* layer = region->getLayer();
  odb::dbChip* chip = region->getChip();
  odb::dbTech* tech = chip->getTech();

  if (!tech) {
    if (auto prop = odb::dbStringProperty::find(chip, "3dblox_tech")) {
      tech = chip->getDb()->findTech(prop->getValue().c_str());
    }
  }
  if (!layer) {
    if (auto prop = odb::dbStringProperty::find(region, "3dblox_layer")) {
      if (tech) {
        layer = tech->findLayer(prop->getValue().c_str());
      }
    }
  }
  if (!layer || !tech) {
    return region->getCuboid();
  }

  uint32_t total_thick = 0, layer_z = 0, target_thick = 0;
  bool reached = false;
  for (odb::dbTechLayer* l : tech->getLayers()) {
    uint32_t t = 0;
    if (l->getType() == odb::dbTechLayerType::ROUTING
        || l->getType() == odb::dbTechLayerType::CUT) {
      if (l->getThickness(t)) {
        total_thick += t;
        if (!reached) {
          layer_z += t;
        }
      }
    }
    if (l == layer) {
      reached = true;
      layer->getThickness(target_thick);
    }
  }

  int z_top, z_bot;
  if (region->getSide() == odb::dbChipRegion::Side::BACK) {
    z_top = layer_z;
    z_bot = layer_z - target_thick;
  } else {
    z_top = std::max(0, (int) chip->getThickness() - (int) (total_thick - layer_z));
    z_bot = z_top - target_thick;
  }
  odb::Rect box = region->getBox();
  return odb::Cuboid(box.xMin(), box.yMin(), z_bot, box.xMax(), box.yMax(), z_top);
}

}  // namespace

namespace odb {

int UnfoldedRegion::getSurfaceZ() const
{
  return isFacingUp() ? cuboid.zMax() : (isFacingDown() ? cuboid.zMin() : cuboid.zCenter());
}

bool UnfoldedRegion::isFacingUp() const { return effective_side == dbChipRegion::Side::FRONT; }
bool UnfoldedRegion::isFacingDown() const { return effective_side == dbChipRegion::Side::BACK; }
bool UnfoldedRegion::isInternal() const { return effective_side == dbChipRegion::Side::INTERNAL; }
bool UnfoldedRegion::isInternalExt() const { return effective_side == dbChipRegion::Side::INTERNAL_EXT; }

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

  bool top_faces_down = top_region->isFacingDown() || top_region->isInternal();
  bool bot_faces_up = bottom_region->isFacingUp() || bottom_region->isInternal();
  bool top_faces_up = top_region->isFacingUp() || top_region->isInternal();
  bool bot_faces_down = bottom_region->isFacingDown() || bottom_region->isInternal();

  bool std_pair = top_faces_down && bot_faces_up;
  bool inv_pair = top_faces_up && bot_faces_down;

  if (!std_pair && !inv_pair) {
    return false;
  }

  int top_z = top_region->getSurfaceZ();
  int bot_z = bottom_region->getSurfaceZ();

  if (top_region->isInternal() || bottom_region->isInternal()) {
    if (std::max(top_region->cuboid.zMin(), bottom_region->cuboid.zMin())
        <= std::min(top_region->cuboid.zMax(), bottom_region->cuboid.zMax())) {
      return true;
    }
    if (std_pair) {
      if (top_region->isInternal()) top_z = top_region->cuboid.zMin();
      if (bottom_region->isInternal()) bot_z = bottom_region->cuboid.zMax();
    } else {
      if (bottom_region->isInternal()) bot_z = bottom_region->cuboid.zMin();
      if (top_region->isInternal()) top_z = top_region->cuboid.zMax();
    }
  }

  if ((std_pair && top_z < bot_z) || (!std_pair && bot_z < top_z)) {
    return false;
  }

  return std::abs(top_z - bot_z) <= connection->getThickness();
}

bool UnfoldedChip::isParentOf(const UnfoldedChip* other) const
{
  if (chip_inst_path.size() >= other->chip_inst_path.size()) {
    return false;
  }
  return std::equal(chip_inst_path.begin(), chip_inst_path.end(), other->chip_inst_path.begin());
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
        if (first == -1) first = idx;
        else uf.unite(first, idx);
      }
    }
  }

  for (const auto& conn : connections) {
    if (!conn.isValid()) continue;
    auto it1 = bumps_by_region.find(conn.top_region);
    auto it2 = bumps_by_region.find(conn.bottom_region);

    if (it1 != bumps_by_region.end() && it2 != bumps_by_region.end()) {
      const auto& idxs1 = it1->second;
      const auto& idxs2 = it2->second;
      if (std::abs(connected_bumps[idxs1[0]]->global_position.z() - 
                   connected_bumps[idxs2[0]]->global_position.z()) <= conn.connection->getThickness()) {
        for (size_t i1 : idxs1) {
          for (size_t i2 : idxs2) {
            if (std::abs(connected_bumps[i1]->global_position.x() - connected_bumps[i2]->global_position.x()) <= bump_pitch_tolerance &&
                std::abs(connected_bumps[i1]->global_position.y() - connected_bumps[i2]->global_position.y()) <= bump_pitch_tolerance) {
              uf.unite(i1, i2);
            }
          }
        }
      }
    }
  }

  std::map<int, std::vector<size_t>> groups;
  for (size_t i = 0; i < connected_bumps.size(); i++) {
    groups[uf.find(i)].push_back(i);
  }
  if (groups.size() <= 1) return {};

  auto max_group = std::max_element(groups.begin(), groups.end(),
      [](auto& a, auto& b) { return a.second.size() < b.second.size(); });

  std::vector<UnfoldedBump*> disconnected;
  for (auto& [root, indices] : groups) {
    if (root != max_group->first) {
      for (size_t idx : indices) disconnected.push_back(connected_bumps[idx]);
    }
  }
  return disconnected;
}

std::string UnfoldedChip::getName() const { return getChipPathKey(chip_inst_path); }

UnfoldedModel::UnfoldedModel(utl::Logger* logger, dbChip* chip) : logger_(logger)
{
  for (dbChipInst* chip_inst : chip->getChipInsts()) {
    std::vector<dbChipInst*> path;
    Cuboid local_cuboid;
    buildUnfoldedChip(chip_inst, path, local_cuboid);
  }
  unfoldConnections(chip);
  unfoldNets(chip);
}

UnfoldedChip* UnfoldedModel::buildUnfoldedChip(dbChipInst* chip_inst,
                                               std::vector<dbChipInst*>& path,
                                               Cuboid& local_cuboid)
{
  dbChip* master_chip = chip_inst->getMasterChip();
  path.push_back(chip_inst);
  UnfoldedChip uf_chip;
  uf_chip.chip_inst_path = path;

  if (master_chip->getChipType() == dbChip::ChipType::HIER) {
    uf_chip.cuboid.mergeInit();
    for (auto sub : master_chip->getChipInsts()) {
      Cuboid sub_local;
      buildUnfoldedChip(sub, path, sub_local);
      uf_chip.cuboid.merge(sub_local);
    }
  } else {
    uf_chip.cuboid = master_chip->getCuboid();
  }

  local_cuboid = uf_chip.cuboid;
  getTransform(chip_inst).apply(local_cuboid);

  dbTransform total;
  for (auto inst : path | std::views::reverse) {
    total.concat(getTransform(inst), total);
  }
  uf_chip.transform = total;
  total.apply(uf_chip.cuboid);
  uf_chip.z_flipped = isPathZFlipped(path);

  for (auto* region_inst : chip_inst->getRegions()) {
    UnfoldedRegion uf_region;
    uf_region.region_inst = region_inst;
    uf_region.effective_side = computeEffectiveSide(region_inst->getChipRegion()->getSide(), path);
    uf_region.cuboid = getCorrectedCuboid(region_inst->getChipRegion());
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
  chip_path_map_[ptr->getName()] = ptr;
  path.pop_back();
  return ptr;
}

void UnfoldedModel::unfoldBumps(UnfoldedRegion& uf_region, const dbTransform& transform)
{
  for (auto* bump_inst : uf_region.region_inst->getChipBumpInsts()) {
    dbChipBump* bump = bump_inst->getChipBump();
    if (!bump->getInst()) continue;
    
    Point global_xy = bump->getInst()->getLocation();
    transform.apply(global_xy);

    UnfoldedBump uf_bump{bump_inst, &uf_region, 
                         Point3D(global_xy.x(), global_xy.y(), uf_region.getSurfaceZ())};

    if (auto prop = dbProperty::find(bump, "logical_net"))
      uf_bump.logical_net_name = ((dbStringProperty*) prop)->getValue();
    
    if (auto prop = dbProperty::find(bump, "logical_port"))
      uf_bump.port_name = ((dbStringProperty*) prop)->getValue();
    else
      uf_bump.port_name = bump_inst->getName();

    uf_region.bumps.push_back(uf_bump);
  }
}

UnfoldedChip* UnfoldedModel::findUnfoldedChip(const std::vector<dbChipInst*>& path)
{
  auto it = chip_path_map_.find(getChipPathKey(path));
  return it != chip_path_map_.end() ? it->second : nullptr;
}

UnfoldedRegion* UnfoldedModel::findUnfoldedRegion(UnfoldedChip* chip, dbChipRegionInst* inst)
{
  if (!chip || !inst) return nullptr;
  auto it = chip->region_map.find(inst);
  return it != chip->region_map.end() ? it->second : nullptr;
}

void UnfoldedModel::unfoldConnections(dbChip* chip) { unfoldConnectionsRecursive(chip, {}); }

void UnfoldedModel::unfoldConnectionsRecursive(dbChip* chip, const std::vector<dbChipInst*>& parent_path)
{
  for (auto* conn : chip->getChipConns()) {
    auto top_path = parent_path;
    for (auto* inst : conn->getTopRegionPath()) top_path.push_back(inst);
    auto bot_path = parent_path;
    for (auto* inst : conn->getBottomRegionPath()) bot_path.push_back(inst);

    UnfoldedRegion* top = findUnfoldedRegion(findUnfoldedChip(top_path), conn->getTopRegion());
    UnfoldedRegion* bot = findUnfoldedRegion(findUnfoldedChip(bot_path), conn->getBottomRegion());

    if (!top && !bot) continue;

    UnfoldedConnection uf_conn{conn, top, bot};
    if (top && bot) uf_conn.connection_cuboid = computeConnectionCuboid(*top, *bot);
    if (top && top->isInternalExt()) top->isUsed = true;
    if (bot && bot->isInternalExt()) bot->isUsed = true;
    unfolded_connections_.push_back(uf_conn);
  }

  for (auto* inst : chip->getChipInsts()) {
    auto sub_path = parent_path;
    sub_path.push_back(inst);
    unfoldConnectionsRecursive(inst->getMasterChip(), sub_path);
  }
}

void UnfoldedModel::unfoldNets(dbChip* chip)
{
  for (auto* net : chip->getChipNets()) {
    UnfoldedNet uf_net{net};
    for (uint32_t i = 0; i < net->getNumBumpInsts(); i++) {
      std::vector<dbChipInst*> path;
      auto it = bump_inst_map_.find(net->getBumpInst(i, path));
      if (it != bump_inst_map_.end()) uf_net.connected_bumps.push_back(it->second);
    }
    unfolded_nets_.push_back(uf_net);
  }
  for (auto* inst : chip->getChipInsts()) unfoldNets(inst->getMasterChip());
}

}  // namespace odb