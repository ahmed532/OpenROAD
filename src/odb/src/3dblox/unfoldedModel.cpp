// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "unfoldedModel.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <ranges>
#include <string>
#include <vector>

#include "odb/db.h"
#include "odb/dbTransform.h"
#include "utl/Logger.h"
#include "utl/unionFind.h"

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
namespace {

static void applyTransform3D(odb::Point3D& point, odb::dbChipInst* inst)
{
  // 1. Apply 2D Transform (XY)
  // Construct 2D transform manually to ensure no 3D artifacts leak in
  odb::Point loc_xy(inst->getLoc().x(), inst->getLoc().y());
  odb::dbTransform t(inst->getOrient().getOrientType2D(), loc_xy);
  t.apply(point);

  // 2. Apply Z Mirror (if any)
  if (inst->getOrient().isMirrorZ()) {
    // "Flip in place" logic: z' = -z + thickness
    // This assumes the mirror happens within the chip's vertical bounding box
    int thickness = inst->getMasterChip()->getThickness();
    point.setZ(-point.z() + thickness);
  }

  // 3. Apply Z Offset
  int z_offset = inst->getLoc().z();
  point.setZ(point.z() + z_offset);
}

static void applyTransform3D(odb::Cuboid& cuboid, odb::dbChipInst* inst)
{
  // 1. Apply 2D Transform (XY)
  odb::Point loc_xy(inst->getLoc().x(), inst->getLoc().y());
  odb::dbTransform t(inst->getOrient().getOrientType2D(), loc_xy);
  t.apply(cuboid);

  // 2. Apply Z Mirror (if any)
  int zlo = cuboid.zMin();
  int zhi = cuboid.zMax();

  if (inst->getOrient().isMirrorZ()) {
    int thickness = inst->getMasterChip()->getThickness();
    zlo = -zlo + thickness;
    zhi = -zhi + thickness;
    std::swap(zlo, zhi);
  }

  // 3. Apply Z Offset
  int z_offset = inst->getLoc().z();
  cuboid.set_zlo(zlo + z_offset);
  cuboid.set_zhi(zhi + z_offset);
}

}  // namespace

namespace odb {

int UnfoldedRegionFull::getSurfaceZ() const
{
  if (isFacingUp()) {
    return cuboid.zMax();
  }
  if (isFacingDown()) {
    return cuboid.zMin();
  }
  return cuboid.zCenter();
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
  if (!top_region->cuboid.xyIntersects(bottom_region->cuboid)) {
    return false;
  }

  // 2. Embedded Chiplet Support (INTERNAL_EXT)
  bool top_is_embedded_host = top_region->isInternalExt();
  bool bottom_is_embedded_host = bottom_region->isInternalExt();

  if (top_is_embedded_host || bottom_is_embedded_host) {
    return true;
  }

  // 3. Connectability Check (Facing Direction & Z Ordering)
  bool top_faces_down = top_region->isFacingDown();
  bool bot_faces_up = bottom_region->isFacingUp();

  bool top_faces_up = top_region->isFacingUp();
  bool bot_faces_down = bottom_region->isFacingDown();

  // Valid Pair 1: Standard (Top chip looking down, Bottom chip looking up)
  bool standard_pair = top_faces_down && bot_faces_up;

  // Valid Pair 2: Inverted/Interposer (Top chip looking up, Bottom chip looking
  // down)
  bool inverted_pair = top_faces_up && bot_faces_down;

  if (!standard_pair && !inverted_pair) {
    return false;
  }

  // 4. Z-Gap & Ordering
  int top_z = top_region->getSurfaceZ();
  int bottom_z = bottom_region->getSurfaceZ();

  if (standard_pair) {
    // Top(Down) should be >= Bot(Up)
    if (top_z < bottom_z) {
      return false;
    }
  } else {
    // Top(Up) and Bot(Down) -> Bot should be >= Top
    if (bottom_z < top_z) {
      return false;
    }
  }

  int gap = std::abs(top_z - bottom_z);
  if (gap > connection->getThickness()) {
    return false;
  }

  return true;
}

std::vector<UnfoldedBump*> UnfoldedNet::getDisconnectedBumps(
    const std::deque<UnfoldedConnection>& connections,
    int bump_pitch_tolerance) const
{
  if (connected_bumps.size() < 2) {
    return {};  // Nothing to check with fewer than 2 bumps
  }

  // Use Union-Find to cluster bumps that are physically aligned.
  // Bumps on the same chip are assumed connected via internal routing.
  // Bumps on different chips are connected if:
  // 1. Their parent regions have a VALID connection.
  // 2. They are physically aligned within tolerance (XY).
  // 3. They are within the connection Z-thickness range.
  utl::UnionFind uf(static_cast<int>(connected_bumps.size()));

  for (size_t i = 0; i < connected_bumps.size(); i++) {
    for (size_t j = i + 1; j < connected_bumps.size(); j++) {
      const auto* b1 = connected_bumps[i];
      const auto* b2 = connected_bumps[j];

      // Same chip: assumed connected via on-die routing
      if (b1->parent_region->parent_chip == b2->parent_region->parent_chip) {
        uf.unite(static_cast<int>(i), static_cast<int>(j));
        continue;
      }

      // Different chips: check for valid region connection
      const UnfoldedConnection* valid_conn = nullptr;
      for (const auto& conn : connections) {
        if (!conn.isValid()) {
          continue;
        }
        bool match = (conn.top_region == b1->parent_region
                      && conn.bottom_region == b2->parent_region)
                     || (conn.top_region == b2->parent_region
                         && conn.bottom_region == b1->parent_region);
        if (match) {
          valid_conn = &conn;
          break;
        }
      }

      if (valid_conn) {
        // Check Geometric Alignment
        int dx = std::abs(b1->global_position.x() - b2->global_position.x());
        int dy = std::abs(b1->global_position.y() - b2->global_position.y());
        int dz = std::abs(b1->global_position.z() - b2->global_position.z());

        if (dx <= bump_pitch_tolerance && dy <= bump_pitch_tolerance
            && dz <= valid_conn->connection->getThickness()) {
          uf.unite(static_cast<int>(i), static_cast<int>(j));
        }
      }
    }
  }

  // Find the largest connected group
  std::map<int, std::vector<size_t>> groups;
  for (size_t i = 0; i < connected_bumps.size(); i++) {
    groups[uf.find(static_cast<int>(i))].push_back(i);
  }

  // If only one group, all bumps are connected
  if (groups.size() <= 1) {
    return {};
  }

  // Find the largest group and return bumps from all other groups
  size_t max_size = 0;
  int max_root = -1;
  for (const auto& [root, indices] : groups) {
    if (indices.size() > max_size) {
      max_size = indices.size();
      max_root = root;
    }
  }

  std::vector<UnfoldedBump*> disconnected;
  for (const auto& [root, indices] : groups) {
    if (root != max_root) {
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
  char delimiter = '/';
  for (dbChipInst* chip_inst : chip_inst_path) {
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

UnfoldedModel::UnfoldedModel(utl::Logger* logger, dbChip* chip)
    : logger_(logger)
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
  path.push_back(chip_inst);
  UnfoldedChip unfolded_chip;
  unfolded_chip.chip_inst_path = path;

  // Initial master cuboid (leaf) or merged sub-instances (HIER)
  if (chip_inst->getMasterChip()->getChipType() == dbChip::ChipType::HIER) {
    unfolded_chip.cuboid.mergeInit();
    for (auto sub_inst : chip_inst->getMasterChip()->getChipInsts()) {
      Cuboid sub_local_cuboid;
      buildUnfoldedChip(sub_inst, path, sub_local_cuboid);
      unfolded_chip.cuboid.merge(sub_local_cuboid);
    }
  } else {
    unfolded_chip.cuboid = chip_inst->getMasterChip()->getCuboid();
  }

  // local_cuboid for parent is this chip's master-coord cuboid transformed by
  // this instance's transform.
  local_cuboid = unfolded_chip.cuboid;
  chip_inst->getTransform().apply(local_cuboid);

  // GLOBAL cuboid for the UnfoldedChip object
  // It starts as the master cuboid (leaf) or merged sub-master cuboid (HIER).
  // We apply the instance transforms of ALL elements in the path from leaf to
  // root.
  for (auto inst : path | std::views::reverse) {
    applyTransform3D(unfolded_chip.cuboid, inst);
  }

  // Unfold regions
  auto chip_type = chip_inst->getMasterChip()->getChipType();
  bool is_simple_type
      = (chip_type == dbChip::ChipType::DIE || chip_type == dbChip::ChipType::IP
         || chip_type == dbChip::ChipType::HIER);

  for (auto region_inst : chip_inst->getRegions()) {
    UnfoldedRegionFull uf_region;
    uf_region.region_inst = region_inst;
    uf_region.parent_chip = nullptr;  // Fix up later
    uf_region.cuboid = region_inst->getChipRegion()->getCuboid();

    // Global coordinates for region FOOTPRINT (XY) and Z
    for (auto inst : path | std::views::reverse) {
      applyTransform3D(uf_region.cuboid, inst);
    }

    auto original_side = region_inst->getChipRegion()->getSide();
    uf_region.effective_side = computeEffectiveSide(original_side, path);

    // Set Z surface using transformed surface point to handle all orientations
    int master_thick = chip_inst->getMasterChip()->getThickness();
    int local_z = 0;
    if (original_side == dbChipRegion::Side::FRONT) {
      local_z = master_thick;
    } else if (original_side == dbChipRegion::Side::BACK) {
      local_z = 0;
    } else {
      local_z = master_thick / 2;  // Approximate for internal
    }

    Point3D surface_pt(0, 0, local_z);
    for (auto inst : path | std::views::reverse) {
      applyTransform3D(surface_pt, inst);
    }
    uf_region.cuboid.set_zlo(surface_pt.z());
    uf_region.cuboid.set_zhi(surface_pt.z());

    // Validation check for Error 411
    if (is_simple_type
        && (uf_region.isInternal() || uf_region.isInternalExt())) {
      logger_->error(
          utl::ODB,
          463,
          "Chip {} of type {} cannot have INTERNAL/INTERNAL_EXT region {}",
          unfolded_chip.getName(),
          chipTypeToString(chip_type),
          region_inst->getChipRegion()->getName());
    }

    unfoldBumps(uf_region, path);
    unfolded_chip.regions.push_back(uf_region);
  }

  unfolded_chips_.push_back(unfolded_chip);
  UnfoldedChip* stable_chip_ptr = &unfolded_chips_.back();
  for (auto& region : stable_chip_ptr->regions) {
    region.parent_chip = stable_chip_ptr;
    for (auto& bump : region.bumps) {
      bump.parent_region = &region;
    }
  }
  chip_path_map_[stable_chip_ptr->getPathKey()] = stable_chip_ptr;

  path.pop_back();
  return stable_chip_ptr;
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
    // Get local position from bump definition
    dbInst* bump_inst = bump->getInst();
    if (!bump_inst) {
      continue;
    }
    Point local_xy = bump_inst->getLocation();

    Point3D global_pos(local_xy.x(), local_xy.y(), 0);
    for (auto inst : path | std::views::reverse) {
      applyTransform3D(global_pos, inst);
    }

    // Z is the region's connecting surface
    global_pos.setZ(uf_region.getSurfaceZ());
    uf_bump.global_position = global_pos;

    // Extract logical net name from property
    dbProperty* net_prop = dbProperty::find(bump, "logical_net");
    if (net_prop && net_prop->getType() == dbProperty::STRING_PROP) {
      uf_bump.logical_net_name = ((dbStringProperty*) net_prop)->getValue();
    }

    // Resolve port name from property
    dbProperty* port_prop = dbProperty::find(bump, "logical_port");
    if (port_prop && port_prop->getType() == dbProperty::STRING_PROP) {
      uf_bump.port_name = ((dbStringProperty*) port_prop)->getValue();
    } else if (uf_bump.bump_inst) {
      uf_bump.port_name = uf_bump.bump_inst->getName();
    }

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

    uf_conn.is_bterm_connection = false;
    if (top_region && bottom_region) {
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
