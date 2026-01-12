// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2026, The OpenROAD Authors

#include "checker.h"

#include "threeDBloxValidator.h"
#include "unfoldedModel.h"

namespace odb {

Checker::Checker(utl::Logger* logger) : logger_(logger)
{
}

void Checker::check(odb::dbChip* chip,
                    int tolerance,
                    int bump_pitch_tolerance,
                    bool verbose,
                    const std::string& report_file)
{
  UnfoldedModel model(logger_);
  model.build(chip);

  ThreeDBloxValidator validator(logger_);
  validator.validate(
      model, chip, tolerance, bump_pitch_tolerance, verbose, report_file);
}

}  // namespace odb
