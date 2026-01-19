// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "unfoldedModel.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <ranges>
#include <string>
#include <unordered_map>
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

// Returns {total_thickness, layer_z}
// layer_z is the z-coordinate of the TOP of the given layer relative to the
// bottom of the stack.
std::pair<int, int> getLayerZ(odb::dbTech* tech, odb::dbTechLayer* target_layer)
{
  int total_thickness = 0;
  int layer_z = 0;
  bool reached_target = false;

  if (!tech) {
    return {0, 0};
  }

  for (auto* layer : tech->getLayers()) {
    auto type = layer->getType();
    if (type == odb::dbTechLayerType::ROUTING
        || type == odb::dbTechLayerType::CUT) {
      uint32_t thick = 0;
      if (layer->getThickness(thick)) {
        total_thickness += thick;
        if (!reached_target) {
          layer_z += thick;
        }
      }
    }
    if (layer == target_layer) {
      reached_target = true;
    }
  }
  return {total_thickness, layer_z};
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

  bool top_is_internal = top_region->isInternal();
  bool bot_is_internal = bottom_region->isInternal();

  // Valid Pair 1: Standard (Top chip looking down, Bottom chip looking up)
  // OR one is INTERNAL (assumed to be connectable from both sides or embedded)
  bool standard_pair = (top_faces_down || top_is_internal)
                       && (bot_faces_up || bot_is_internal);

  // Valid Pair 2: Inverted/Interposer (Top chip looking up, Bottom chip looking
  // down)
  bool inverted_pair = (top_faces_up || top_is_internal)
                       && (bot_faces_down || bot_is_internal);

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
    utl::Logger* logger,
    const std::deque<UnfoldedConnection>& connections,
    int bump_pitch_tolerance) const
{
  if (connected_bumps.size() < 2) {
    return {};
  }

  utl::UnionFind uf(static_cast<int>(connected_bumps.size()));

  // 1. Group bumps by region
  std::map<UnfoldedRegion*, std::vector<size_t>> bumps_by_region;
  for (size_t i = 0; i < connected_bumps.size(); i++) {
    bumps_by_region[connected_bumps[i]->parent_region].push_back(i);
  }

  // 2. Group regions by chip to unite bumps on the same chip
  std::map<UnfoldedChip*, std::vector<UnfoldedRegion*>> regions_by_chip;
  for (auto& [region, _] : bumps_by_region) {
    regions_by_chip[region->parent_chip].push_back(region);
  }

  for (auto& [chip, regions] : regions_by_chip) {
    int first_idx = -1;
    for (auto* region : regions) {
      for (size_t idx : bumps_by_region.at(region)) {
        if (first_idx == -1) {
          first_idx = static_cast<int>(idx);
        } else {
          uf.unite(first_idx, static_cast<int>(idx));
        }
      }
    }
  }

  // 3. Check connectivity between regions across chips
  for (const auto& conn : connections) {
    if (!conn.isValid()) {
      continue;
    }
    auto it1 = bumps_by_region.find(conn.top_region);
    auto it2 = bumps_by_region.find(conn.bottom_region);

    if (it1 != bumps_by_region.end() && it2 != bumps_by_region.end()) {
      const auto& idxs1 = it1->second;
      const auto& idxs2 = it2->second;

      // Check Z distance between regions
      int dz = std::abs(connected_bumps[idxs1[0]]->global_position.z()
                        - connected_bumps[idxs2[0]]->global_position.z());

      if (dz <= conn.connection->getThickness()) {
        for (size_t i1 : idxs1) {
          const auto& p1 = connected_bumps[i1]->global_position;
          for (size_t i2 : idxs2) {
            const auto& p2 = connected_bumps[i2]->global_position;
            if (std::abs(p1.x() - p2.x()) <= bump_pitch_tolerance
                && std::abs(p1.y() - p2.y()) <= bump_pitch_tolerance) {
              uf.unite(static_cast<int>(i1), static_cast<int>(i2));
            }
          }
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

  // local_cuboid for parent is this chip's master-coord cuboid transformed
  // by this instance's transform.
  local_cuboid = unfolded_chip.cuboid;
  chip_inst->getTransform().apply(local_cuboid);

  // GLOBAL cuboid for the UnfoldedChip object
  // It starts as the master cuboid (leaf) or merged sub-master cuboid
  // (HIER). We apply the instance transforms of ALL elements in the path
  // from leaf to root.
  for (auto inst : path | std::views::reverse) {
    applyTransform3D(unfolded_chip.cuboid, inst);
  }

  // Unfold regions
  auto chip_type = chip_inst->getMasterChip()->getChipType();
  bool is_simple_type = (chip_type == dbChip::ChipType::DIE
                         || chip_type == dbChip::ChipType::IP);

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

    // Set Z surface using transformed surface point to handle all
    // orientations
    int master_thick = chip_inst->getMasterChip()->getThickness();
    int local_z = 0;
    if (original_side == dbChipRegion::Side::FRONT) {
      local_z = master_thick;
    } else if (original_side == dbChipRegion::Side::BACK) {
      local_z = 0;
    } else {
      // Calculate specific Z for internal / internal_ext layers
      auto* region = region_inst->getChipRegion();
      auto* layer = region->getLayer();

      bool z_set = false;
      if (layer) {
        auto* tech = chip_inst->getMasterChip()->getTech();
        if (tech) {
          auto [total_thick, layer_z] = getLayerZ(tech, layer);
          // Assuming BEOL is at the "FRONT" (top) of the chip
          // layer_z is distance from bottom of BEOL stack to top of layer
          // total_thick is height of BEOL stack.
          // So layer is (total_thick - layer_z) from the TOP of BEOL.
          // If BEOL is on top of substrate (master_thick), then:
          // top of layer Z = master_thick - (total_thick - layer_z)

          if (total_thick > 0) {
            int dist_from_top = total_thick - layer_z;
            local_z = master_thick - dist_from_top;
            // Clamp within chip body
            if (local_z < 0) {
              local_z = 0;
            }
            if (local_z > master_thick) {
              local_z = master_thick;
            }
            z_set = true;
          }
        }
      }

      if (!z_set) {
        local_z = master_thick / 2;  // Approximate for internal
      }
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
          unfolded_chip.getName().c_str(),
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
    stable_chip_ptr->region_map[region.region_inst] = &region;
    for (auto& bump : region.bumps) {
      bump.parent_region = &region;
      bump_inst_map_[bump.bump_inst] = &bump;
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

  auto bumps = region->getChipBumps();

  // Pre-map bump instances to their definitions for O(1) lookup
  std::unordered_map<dbChipBump*, dbChipBumpInst*> bump_to_inst;
  for (auto* bi : uf_region.region_inst->getChipBumpInsts()) {
    bump_to_inst[bi->getChipBump()] = bi;
  }

  for (auto* bump : bumps) {
    UnfoldedBump uf_bump;
    uf_bump.bump_inst = nullptr;
    auto it = bump_to_inst.find(bump);
    if (it != bump_to_inst.end()) {
      uf_bump.bump_inst = it->second;
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
  if (!chip || !region_inst) {
    return nullptr;
  }
  auto it = chip->region_map.find(region_inst);
  if (it != chip->region_map.end()) {
    return it->second;
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

      auto it = bump_inst_map_.find(bump_inst);
      if (it != bump_inst_map_.end()) {
        uf_net.connected_bumps.push_back(it->second);
      }
    }
    unfolded_nets_.push_back(uf_net);
  }

  for (auto* inst : chip->getChipInsts()) {
    unfoldNets(inst->getMasterChip());
  }
}

}  // namespace odb
