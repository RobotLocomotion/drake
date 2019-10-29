#pragma once

/// @file
/// This file defines gflags settings to control spdlog levels.
/// Only include this from translation units that declare a `main` function.

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"

// Declare the --spdlog_level gflags option.
DEFINE_string(spdlog_level,
              drake::logging::kSetLogLevelUnchanged,
              drake::logging::kSetLogLevelHelpMessage);

namespace drake {
namespace logging {

/// Check the gflags settings for validity.  If spdlog is enabled, update its
/// configuration to match the flags.
inline void HandleSpdlogGflags() {
  drake::logging::set_log_level(FLAGS_spdlog_level);
}

}  // namespace logging
}  // namespace drake
