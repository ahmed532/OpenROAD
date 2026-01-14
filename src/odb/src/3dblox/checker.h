// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#pragma once

#include "odb/db.h"
#include "odb/geom.h"
#include "unfoldedModel.h"
#include "utl/Logger.h"

namespace sta {
class Sta;
}

namespace odb {
class dbChip;
class dbMarkerCategory;

class Checker
{
 public:
  Checker(utl::Logger* logger);
  ~Checker() = default;
  void check(dbChip* chip, int bump_pitch_tolerance = 1);

 private:
  void checkFloatingChips(const UnfoldedModel& model,
                          dbMarkerCategory* category);
  void checkOverlappingChips(const UnfoldedModel& model,
                             dbMarkerCategory* category);
  void checkConnectionRegions(const UnfoldedModel& model,
                              dbChip* chip,
                              dbMarkerCategory* category);
  void checkNetConnectivity(const UnfoldedModel& model,
                            dbChip* chip,
                            dbMarkerCategory* category,
                            int bump_pitch_tolerance);

  void checkInternalExtUsage(const UnfoldedModel& model,
                             dbMarkerCategory* category);
  void checkConnectivity(odb::dbChip* chip, odb::dbMarkerCategory* category);
  void checkLogicalAlignment(odb::dbChip* chip,
                             odb::dbMarkerCategory* category);

  bool isOverlapFullyInConnections(const UnfoldedModel& model,
                                   const UnfoldedChip* chip1,
                                   const UnfoldedChip* chip2,
                                   const Cuboid& overlap) const;
  void checkBumpPhysicalAlignment(const UnfoldedModel& model,
                                  dbMarkerCategory* category);
  utl::Logger* logger_;
};

}  // namespace odb
