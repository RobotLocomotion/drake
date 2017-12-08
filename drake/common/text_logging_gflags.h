#pragma once

/// @file
/// This file defines gflags settings to control spdlog levels.
/// Only include this from translation units that declare a `main` function.

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"

// Declare the --spdlog_level gflags option.
DEFINE_string(spdlog_level, "unchanged",
              "sets the spdlog output threshold; "
              "possible values are 'unchanged', 'trace', 'debug', 'warn'");

namespace drake {
namespace logging {

/// Check the gflags settings for validity.  If spdlog is enabled, update its
/// configuration to match the flags.
inline void HandleSpdlogGflags() {
  const bool want_unchanged = (FLAGS_spdlog_level == "unchanged");
  const bool want_trace = (FLAGS_spdlog_level == "trace");
  const bool want_debug = (FLAGS_spdlog_level == "debug");
  const bool want_warn = (FLAGS_spdlog_level == "warn");
  if (!want_unchanged && !want_trace && !want_debug && !want_warn) {
    log()->critical("Unknown spdlog_level {}", FLAGS_spdlog_level);
    throw std::runtime_error("Unknown spdlog level");
  }
#ifdef HAVE_SPDLOG
  if (want_trace) {
    log()->set_level(spdlog::level::trace);
  } else if (want_debug) {
    log()->set_level(spdlog::level::debug);
  } else if (want_warn) {
    log()->set_level(spdlog::level::warn);
  }
#endif
}

}  // namespace logging
}  // namespace drake
