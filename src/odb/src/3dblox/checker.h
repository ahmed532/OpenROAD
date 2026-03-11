// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#pragma once

#include <map>
#include <string>

#include "odb/3dblox.h"
#include "odb/db.h"
#include "odb/unfoldedModel.h"
#include "utl/Logger.h"

namespace sta {
class Sta;
}

namespace odb {
class dbChip;
class dbMarkerCategory;

struct MatingSurfaces
{
  bool valid;
  int top_z;
  int bot_z;
};

class Checker
{
 public:
  Checker(utl::Logger* logger);
  ~Checker() = default;
  void check(dbChip* chip,
             const std::map<std::string, PathAssertion>& path_assertions);

 private:
  void checkLogicalConnectivity(dbMarkerCategory* top_cat,
                                const UnfoldedModel& model);
  void checkFloatingChips(dbMarkerCategory* top_cat,
                          const UnfoldedModel& model);
  void checkOverlappingChips(dbMarkerCategory* top_cat,
                             const UnfoldedModel& model);
  void checkInternalExtUsage(dbMarkerCategory* top_cat,
                             const UnfoldedModel& model);
  void checkConnectionRegions(dbMarkerCategory* top_cat,
                              const UnfoldedModel& model);
  void checkBumpPhysicalAlignment(dbMarkerCategory* top_cat,
                                  const UnfoldedModel& model);
  void checkPathAssertions(
      dbMarkerCategory* top_cat,
      const UnfoldedModel& model,
      const std::map<std::string, PathAssertion>& path_assertions);
  utl::Logger* logger_;
};

}  // namespace odb
