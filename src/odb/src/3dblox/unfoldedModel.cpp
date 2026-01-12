// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "unfoldedModel.h"

#include <ranges>
#include <string>
#include <vector>

#include "odb/db.h"
#include "odb/dbTransform.h"
#include "utl/Logger.h"

namespace {

odb::dbChipRegion::Side computeEffectiveSide(
    odb::dbChipRegion::Side original,
    const std::vector<odb::dbChipInst*>& path)
{
  bool z_flipped = false;
  for (auto inst : path) {
    if (inst->getOrient().isMirrorZ()) {
      z_flipped = !z_flipped;
    }
  }

  if (!z_flipped) {
    return original;
  }

  switch (original) {
    case odb::dbChipRegion::Side::FRONT:
      return odb::dbChipRegion::Side::BACK;
    case odb::dbChipRegion::Side::BACK:
      return odb::dbChipRegion::Side::FRONT;
    default:
      return original;
  }
}

odb::Cuboid computeConnectionCuboid(const odb::UnfoldedRegionFull& top,
                                    const odb::UnfoldedRegionFull& bottom,
                                    int thickness)
{
  odb::Cuboid result;
  result.set_xlo(std::max(top.cuboid.xMin(), bottom.cuboid.xMin()));
  result.set_ylo(std::max(top.cuboid.yMin(), bottom.cuboid.yMin()));
  result.set_xhi(std::min(top.cuboid.xMax(), bottom.cuboid.xMax()));
  result.set_yhi(std::min(top.cuboid.yMax(), bottom.cuboid.yMax()));

  int z_min = std::min(top.getSurfaceZ(), bottom.getSurfaceZ());
  int z_max = std::max(top.getSurfaceZ(), bottom.getSurfaceZ());

  result.set_zlo(z_min);
  result.set_zhi(z_max);
  return result;
}

std::string chipTypeToString(odb::dbChip::ChipType type)
{
  switch (type) {
    case odb::dbChip::ChipType::DIE:
      return "DIE";
    case odb::dbChip::ChipType::RDL:
      return "RDL";
    case odb::dbChip::ChipType::IP:
      return "IP";
    case odb::dbChip::ChipType::SUBSTRATE:
      return "SUBSTRATE";
    case odb::dbChip::ChipType::HIER:
      return "HIER";
  }
  return "UNKNOWN";
}

}  // namespace

namespace odb {

int UnfoldedRegionFull::getSurfaceZ() const
{
  switch (effective_side) {
    case dbChipRegion::Side::FRONT:
      return cuboid.zMax();
    case dbChipRegion::Side::BACK:
      return cuboid.zMin();
    default:
      return cuboid.zCenter();  // Internal
  }
}

bool UnfoldedRegionFull::isFacingUp() const
{
  return effective_side == dbChipRegion::Side::FRONT;
}

bool UnfoldedRegionFull::isFacingDown() const
{
  return effective_side == dbChipRegion::Side::BACK;
}

bool UnfoldedRegionFull::isInternal() const
{
  return effective_side == dbChipRegion::Side::INTERNAL;
}

bool UnfoldedRegionFull::isInternalExt() const
{
  return effective_side == dbChipRegion::Side::INTERNAL_EXT;
}

bool UnfoldedConnection::isValid() const
{
  if (is_bterm_connection) {
    return true;  // BTerms valid by logic
  }
  // Virtual connections (null regions) are not geometrically validated
  if (!top_region || !bottom_region) {
    return true;  // Considered valid by definition
  }

  // 1. XY Overlap
  if (!top_region->cuboid.xyOverlaps(bottom_region->cuboid)) {
    return false;
  }

  // 2. Embedded Chiplet Support (INTERNAL_EXT)
  bool top_is_embedded_host = top_region->isInternalExt();
  bool bottom_is_embedded_host = bottom_region->isInternalExt();

  if (top_is_embedded_host || bottom_is_embedded_host) {
    // Geometric containment check: The embedded chip (FRONT/BACK)
    // must be spatially contained or overlapping with the host's INTERNAL_EXT
    // region. We relax "facing" rules for embedded.
    return true;
  }

  // 3. Standard & Interposer Facing Logic
  bool top_down = top_region->isFacingDown();
  bool bot_up = bottom_region->isFacingUp();
  bool top_up = top_region->isFacingUp();
  bool bot_down = bottom_region->isFacingDown();

  // Standard: Top(Back/Down) <-> Bottom(Front/Up)
  // Interposer: Top(Front/Up) <-> Bottom(Back/Down) [Inverted]
  if (!(top_down && bot_up) && !(top_up && bot_down)) {
    return false;
  }

  // 4. Z-Gap
  int top_z = top_region->getSurfaceZ();
  int bottom_z = bottom_region->getSurfaceZ();
  if (top_z < bottom_z) {
    std::swap(top_z, bottom_z);
  }

  return (top_z - bottom_z) <= connection->getThickness();
}

std::vector<UnfoldedBump*> UnfoldedNet::getDisconnectedBumps(
    int bump_pitch_tolerance) const
{
  // This logic will be handled by ThreeDBloxValidator using Union-Find
  return {};
}

std::string UnfoldedChip::getName() const
{
  std::string name;
  char delimiter = '/';
  if (!chip_inst_path.empty()) {
    dbBlock* block = chip_inst_path[0]->getParentChip()->getBlock();
    if (block) {
      char d = block->getHierarchyDelimiter();
      if (d != 0) {
        delimiter = d;
      }
    }
  }
  for (auto* chip_inst : chip_inst_path) {
    if (!name.empty()) {
      name += delimiter;
    }
    name += chip_inst->getName();
  }
  return name;
}

std::string UnfoldedChip::getPathKey() const
{
  return getName();
}

UnfoldedModel::UnfoldedModel(utl::Logger* logger) : logger_(logger)
{
}

void UnfoldedModel::build(dbChip* chip)
{
  for (dbChipInst* chip_inst : chip->getChipInsts()) {
    UnfoldedChip unfolded_chip;
    unfoldChip(chip_inst, unfolded_chip);
  }
  unfoldConnections(chip);
  unfoldNets(chip);
}

void UnfoldedModel::unfoldChip(dbChipInst* chip_inst,
                               UnfoldedChip& unfolded_chip)
{
  unfolded_chip.chip_inst_path.push_back(chip_inst);

  if (chip_inst->getMasterChip()->getChipType() == dbChip::ChipType::HIER) {
    for (auto sub_inst : chip_inst->getMasterChip()->getChipInsts()) {
      unfoldChip(sub_inst, unfolded_chip);
    }
  } else {
    // Original logic: Leaf chip cuboid calculation
    unfolded_chip.cuboid = chip_inst->getMasterChip()->getCuboid();
    for (auto inst : unfolded_chip.chip_inst_path | std::views::reverse) {
      inst->getTransform().apply(unfolded_chip.cuboid);
    }

    // Unfold regions
    auto chip_type = chip_inst->getMasterChip()->getChipType();
    bool is_simple_type = (chip_type == dbChip::ChipType::DIE
                           || chip_type == dbChip::ChipType::IP
                           || chip_type == dbChip::ChipType::HIER);

    for (auto region_inst : chip_inst->getRegions()) {
      UnfoldedRegionFull uf_region;
      uf_region.region_inst = region_inst;
      uf_region.parent_chip = &unfolded_chip;  // This is temporary pointer
      uf_region.cuboid = region_inst->getChipRegion()->getCuboid();

      // Apply hierarchy transforms to region cuboid
      for (auto inst : unfolded_chip.chip_inst_path | std::views::reverse) {
        inst->getTransform().apply(uf_region.cuboid);
      }

      // Compute effective side after all flips
      uf_region.effective_side
          = computeEffectiveSide(region_inst->getChipRegion()->getSide(),
                                 unfolded_chip.chip_inst_path);

      // Validation check for Error 411
      if (is_simple_type
          && (uf_region.isInternal() || uf_region.isInternalExt())) {
        logger_->error(utl::ODB,
                       463,
                       "Chip {} of type {} cannot have INTERNAL/INTERNAL_EXT "
                       "region {}",
                       unfolded_chip.getName().c_str(),
                       chipTypeToString(chip_type).c_str(),
                       region_inst->getChipRegion()->getName());
      }

      // Unfold bumps
      unfoldBumps(uf_region, unfolded_chip.chip_inst_path);

      unfolded_chip.regions.push_back(uf_region);
    }

    // Fix up parent pointers after all regions are added (deque stability
    // check) Note: Since unfolded_chip is local here and then copied into the
    // deque, the regions inside will be copied. However, the `parent_chip`
    // pointer inside `uf_region` needs to point to the stable location in
    // `unfolded_chips_`. We handle this AFTER pushing back to
    // `unfolded_chips_`.

    unfolded_chips_.push_back(unfolded_chip);
    UnfoldedChip* stable_chip_ptr = &unfolded_chips_.back();
    for (auto& region : stable_chip_ptr->regions) {
      region.parent_chip = stable_chip_ptr;
      for (auto& bump : region.bumps) {
        bump.parent_region = &region;
      }
    }

    chip_path_map_[stable_chip_ptr->getPathKey()] = stable_chip_ptr;
  }

  unfolded_chip.chip_inst_path.pop_back();
}

void UnfoldedModel::unfoldBumps(UnfoldedRegionFull& uf_region,
                                const std::vector<dbChipInst*>& path)
{
  dbChipRegion* region = uf_region.region_inst->getChipRegion();

  for (auto* bump : region->getChipBumps()) {
    UnfoldedBump uf_bump;
    uf_bump.bump_inst = nullptr;
    for (auto* bi : uf_region.region_inst->getChipBumpInsts()) {
      if (bi->getChipBump() == bump) {
        uf_bump.bump_inst = bi;
        break;
      }
    }
    uf_bump.parent_region = &uf_region;

    // Get local position from bump definition
    Point local_xy = bump->getInst()->getLocation();

    // Transform to global coordinates
    Point3D global_pos(local_xy.x(), local_xy.y(), 0);
    for (auto inst : path | std::views::reverse) {
      inst->getTransform().apply(global_pos);
    }

    // Z is the region's connecting surface
    global_pos.setZ(uf_region.getSurfaceZ());
    uf_bump.global_position = global_pos;

    // Extract logical net name from property
    dbProperty* prop = dbProperty::find(bump, "logical_net");
    if (prop && prop->getType() == dbProperty::STRING_PROP) {
      uf_bump.logical_net_name = ((dbStringProperty*) prop)->getValue();
    }
    // TODO: Get port name from where? The bmap isn't directly stored on object
    // easily. Assuming bump inst name for now or handled elsewhere.

    uf_region.bumps.push_back(uf_bump);
  }
}

UnfoldedChip* UnfoldedModel::findUnfoldedChip(
    const std::vector<dbChipInst*>& path)
{
  std::string key;
  char delimiter = '/';
  if (!path.empty()) {
    dbBlock* block = path[0]->getParentChip()->getBlock();
    if (block) {
      char d = block->getHierarchyDelimiter();
      if (d != 0) {
        delimiter = d;
      }
    }
  }

  for (size_t i = 0; i < path.size(); i++) {
    key += path[i]->getName();
    if (i < path.size() - 1) {
      key += delimiter;
    }
  }

  auto it = chip_path_map_.find(key);
  if (it != chip_path_map_.end()) {
    return it->second;
  }
  return nullptr;
}

UnfoldedRegionFull* UnfoldedModel::findUnfoldedRegion(
    UnfoldedChip* chip,
    dbChipRegionInst* region_inst)
{
  if (!chip) {
    return nullptr;
  }
  for (auto& region : chip->regions) {
    if (region.region_inst == region_inst) {
      return &region;
    }
  }
  return nullptr;
}

void UnfoldedModel::unfoldConnections(dbChip* chip)
{
  unfoldConnectionsRecursive(chip, {});
}

void UnfoldedModel::unfoldConnectionsRecursive(
    dbChip* chip,
    const std::vector<dbChipInst*>& parent_path)
{
  for (auto* conn : chip->getChipConns()) {
    auto* top_region_inst = conn->getTopRegion();
    auto* bottom_region_inst = conn->getBottomRegion();

    // Check for BTerm connection
    // If top or bottom chip is not resolved, it might be a BTerm connection?
    // The dbChipConn model:
    // If top_region_inst is null, it might be virtual.
    // But how do we distinguish BTerm connection from purely virtual?
    // Assuming for now if one side is missing, it's potentially BTerm or
    // virtual.

    auto top_full_path = parent_path;
    for (auto* inst : conn->getTopRegionPath()) {
      top_full_path.push_back(inst);
    }
    UnfoldedChip* top_chip = findUnfoldedChip(top_full_path);
    UnfoldedRegionFull* top_region
        = findUnfoldedRegion(top_chip, top_region_inst);

    auto bottom_full_path = parent_path;
    for (auto* inst : conn->getBottomRegionPath()) {
      bottom_full_path.push_back(inst);
    }
    UnfoldedChip* bottom_chip = findUnfoldedChip(bottom_full_path);
    UnfoldedRegionFull* bottom_region
        = findUnfoldedRegion(bottom_chip, bottom_region_inst);

    if (!top_region && !bottom_region) {
      continue;  // Skip totally virtual/unresolved
    }

    UnfoldedConnection uf_conn;
    uf_conn.connection = conn;
    uf_conn.top_region = top_region;
    uf_conn.bottom_region = bottom_region;
    // Pointers to chips for checks (redundant with region->parent_chip but kept
    // for now)

    // Check if one side is BTerm (TODO: verify how BTerm connections are
    // represented in dbChipConn) Currently dbChipConn doesn't explicitly point
    // to dbBTerm. If one side is null, we assume it's external/BTerm for now
    // and valid.
    if (!top_region || !bottom_region) {
      uf_conn.is_bterm_connection = true;
    } else {
      uf_conn.connection_cuboid = computeConnectionCuboid(
          *top_region, *bottom_region, conn->getThickness());
    }

    // Mark used for INTERNAL_EXT check
    if (top_region && top_region->isInternalExt()) {
      top_region->isUsed = true;
    }
    if (bottom_region && bottom_region->isInternalExt()) {
      bottom_region->isUsed = true;
    }

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
    UnfoldedNet uf_net;
    uf_net.chip_net = net;

    for (uint32_t i = 0; i < net->getNumBumpInsts(); i++) {
      std::vector<dbChipInst*> path;
      dbChipBumpInst* bump_inst = net->getBumpInst(i, path);

      // Resolve path to UnfoldedChip
      UnfoldedChip* uf_chip = findUnfoldedChip(path);
      if (!uf_chip) {
        continue;
      }

      // Find UnfoldedRegion that contains this bump inst
      UnfoldedRegionFull* uf_region
          = findUnfoldedRegion(uf_chip, bump_inst->getChipRegionInst());
      if (!uf_region) {
        continue;
      }

      // Find UnfoldedBump
      for (auto& bump : uf_region->bumps) {
        if (bump.bump_inst == bump_inst) {
          uf_net.connected_bumps.push_back(&bump);
          break;
        }
      }
    }
    unfolded_nets_.push_back(uf_net);
  }

  for (auto* inst : chip->getChipInsts()) {
    unfoldNets(inst->getMasterChip());
  }
}

}  // namespace odb
