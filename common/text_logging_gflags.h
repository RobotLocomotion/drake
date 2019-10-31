#pragma once

/// @file
/// This file defines gflags settings to control spdlog levels.
/// Only include this from translation units that declare a `main` function.

// NOLINTNEXTLINE(whitespace/line_length)
#warning DRAKE DEPRECATED: The drake/common/text_logging_gflags.h convenience header is being removed from Drake on or after 2020-02-01.  Downstream projects should maintain their own implementation of this feature.  See drake/common/add_text_logging_gflags.cc for an example.

#include <gflags/gflags.h>

#include "drake/common/drake_deprecated.h"
#include "drake/common/text_logging.h"

// Declare the --spdlog_level gflags option.
DEFINE_string(spdlog_level,
              drake::logging::kSetLogLevelUnchanged,
              drake::logging::kSetLogLevelHelpMessage);

namespace drake {
namespace logging {

/// Check the gflags settings for validity.  If spdlog is enabled, update its
/// configuration to match the flags.
DRAKE_DEPRECATED("2020-02-01", "The drake/common/text_logging_gflags.h convenience header is being removed from Drake.  Downstream projects should maintain their own implementation of this feature.  See drake/common/add_text_logging_gflags.cc for an example.")  // NOLINT(whitespace/line_length)
inline void HandleSpdlogGflags() {
  drake::logging::set_log_level(FLAGS_spdlog_level);
}

}  // namespace logging
}  // namespace drake
