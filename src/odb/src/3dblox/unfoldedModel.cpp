// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "unfoldedModel.h"

#include <algorithm>
#include <deque>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "odb/db.h"
#include "odb/dbTransform.h"
#include "odb/geom.h"
#include "utl/Logger.h"

namespace {

std::vector<odb::dbChipInst*> concatPath(
    const std::vector<odb::dbChipInst*>& head,
    const std::vector<odb::dbChipInst*>& tail)
{
  std::vector<odb::dbChipInst*> full = head;
  full.insert(full.end(), tail.begin(), tail.end());
  return full;
}

odb::dbTech* getTech(odb::dbChip* chip)
{
  if (auto* tech = chip->getTech()) {
    return tech;
  }
  if (auto prop = odb::dbStringProperty::find(chip, "3dblox_tech")) {
    return chip->getDb()->findTech(prop->getValue().c_str());
  }
  return nullptr;
}

std::string getFullPathName(const std::vector<odb::dbChipInst*>& path)
{
  std::string name;
  for (auto* p : path) {
    if (!name.empty()) {
      name += '/';
    }
    name += p->getName();
  }
  return name;
}

}  // namespace

namespace odb {

int UnfoldedRegion::getSurfaceZ() const
{
  return isFront() ? cuboid.zMax()
                   : (isBack() ? cuboid.zMin() : cuboid.zCenter());
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

  const bool top_faces_down = top_region->isBack() || top_region->isInternal();
  const bool top_faces_up = top_region->isFront() || top_region->isInternal();
  const bool bot_faces_down
      = bottom_region->isBack() || bottom_region->isInternal();
  const bool bot_faces_up
      = bottom_region->isFront() || bottom_region->isInternal();

  bool standard_pair = top_faces_down && bot_faces_up;
  bool inverted_pair = top_faces_up && bot_faces_down;
  if (!standard_pair && !inverted_pair) {
    return false;
  }

  const Cuboid& t_cub = top_region->cuboid;
  const Cuboid& b_cub = bottom_region->cuboid;

  if ((top_region->isInternal() || bottom_region->isInternal())
      && std::max(t_cub.zMin(), b_cub.zMin())
             <= std::min(t_cub.zMax(), b_cub.zMax())) {
    return true;
  }

  int t_z = top_region->getSurfaceZ();
  int b_z = bottom_region->getSurfaceZ();

  if (standard_pair && inverted_pair && t_z < b_z) {
    std::swap(standard_pair, inverted_pair);
  }

  if (standard_pair) {
    if (top_region->isInternal()) {
      t_z = t_cub.zMin();
    }
    if (bottom_region->isInternal()) {
      b_z = b_cub.zMax();
    }
    if (t_z < b_z) {
      return false;
    }
  } else {
    if (bottom_region->isInternal()) {
      b_z = b_cub.zMin();
    }
    if (top_region->isInternal()) {
      t_z = t_cub.zMax();
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

UnfoldedModel::UnfoldedModel(utl::Logger* logger, dbChip* chip)
    : logger_(logger)
{
  for (dbChipInst* inst : chip->getChipInsts()) {
    std::vector<dbChipInst*> path;
    Cuboid local;
    buildUnfoldedChip(inst, path, dbTransform(), local);
  }
  unfoldConnections(chip, {});
  unfoldNets(chip, {});
}

UnfoldedChip* UnfoldedModel::buildUnfoldedChip(dbChipInst* inst,
                                               std::vector<dbChipInst*>& path,
                                               const dbTransform& parent_xform,
                                               Cuboid& local)
{
  dbChip* master = inst->getMasterChip();
  path.push_back(inst);

  dbTransform inst_xform = inst->getTransform();
  dbTransform total = inst_xform;
  total.concat(parent_xform);

  dbTech* tech = getTech(master);

  UnfoldedChip uf_chip{
      .name = getFullPathName(path), .tech = tech, .chip_inst_path = path};

  if (master->getChipType() == dbChip::ChipType::HIER) {
    uf_chip.cuboid.mergeInit();
    for (auto sub : master->getChipInsts()) {
      Cuboid sub_local;
      buildUnfoldedChip(sub, path, total, sub_local);
      uf_chip.cuboid.merge(sub_local);
    }
  } else {
    uf_chip.cuboid = master->getCuboid();
  }

  local = uf_chip.cuboid;
  inst_xform.apply(local);

  uf_chip.transform = total;
  total.apply(uf_chip.cuboid);

  unfoldRegions(uf_chip, inst, total);

  unfolded_chips_.push_back(std::move(uf_chip));
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

  unfoldConnections(master, path);
  unfoldNets(master, path);

  path.pop_back();
  return ptr;
}

void UnfoldedModel::unfoldRegions(UnfoldedChip& uf_chip,
                                  dbChipInst* inst,
                                  const dbTransform& transform)
{
  for (auto* region_inst : inst->getRegions()) {
    auto side = region_inst->getChipRegion()->getSide();
    if (uf_chip.transform.isMirrorZ()) {
      if (side == dbChipRegion::Side::FRONT) {
        side = dbChipRegion::Side::BACK;
      } else if (side == dbChipRegion::Side::BACK) {
        side = dbChipRegion::Side::FRONT;
      }
    }

    UnfoldedRegion uf_region{
        .region_inst = region_inst,
        .effective_side = side,
        .cuboid = region_inst->getChipRegion()->getCuboid()};
    transform.apply(uf_region.cuboid);
    unfoldBumps(uf_region, transform);
    uf_chip.regions.push_back(std::move(uf_region));
  }
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

void UnfoldedModel::unfoldConnections(
    dbChip* chip,
    const std::vector<dbChipInst*>& parent_path)
{
  for (auto* conn : chip->getChipConns()) {
    UnfoldedRegion* top = findUnfoldedRegion(
        findUnfoldedChip(concatPath(parent_path, conn->getTopRegionPath())),
        conn->getTopRegion());
    UnfoldedRegion* bot = findUnfoldedRegion(
        findUnfoldedChip(concatPath(parent_path, conn->getBottomRegionPath())),
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
}

void UnfoldedModel::unfoldNets(dbChip* chip,
                               const std::vector<dbChipInst*>& parent_path)
{
  for (auto* net : chip->getChipNets()) {
    UnfoldedNet uf_net{.chip_net = net};
    for (uint32_t i = 0; i < net->getNumBumpInsts(); i++) {
      std::vector<dbChipInst*> rel_path;
      dbChipBumpInst* b_inst = net->getBumpInst(i, rel_path);

      auto it = bump_inst_map_.find(b_inst);
      if (it != bump_inst_map_.end()) {
        uf_net.connected_bumps.push_back(it->second);
      }
    }
    unfolded_nets_.push_back(std::move(uf_net));
  }
}

}  // namespace odb
