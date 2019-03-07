#pragma once

/// @file
/// This file defines gflags settings to control spdlog levels.
/// Only include this from translation units that declare a `main` function.

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"

// Declare the --spdlog_level gflags option.
DEFINE_string(spdlog_level, "unchanged",
              "sets the spdlog output threshold; possible values are "
              "'unchanged', "
              "'trace', "
              "'debug', "
              "'info', "
              "'warn', "
              "'err', "
              "'critical', "
              "'off'");

namespace drake {
namespace logging {

/// Check the gflags settings for validity.  If spdlog is enabled, update its
/// configuration to match the flags.
inline void HandleSpdlogGflags() {
  const bool want_unchanged = (FLAGS_spdlog_level == "unchanged");
  const bool want_trace = (FLAGS_spdlog_level == "trace");
  const bool want_debug = (FLAGS_spdlog_level == "debug");
  const bool want_info = (FLAGS_spdlog_level == "info");
  const bool want_warn = (FLAGS_spdlog_level == "warn");
  const bool want_err = (FLAGS_spdlog_level == "err");
  const bool want_critical = (FLAGS_spdlog_level == "critical");
  const bool want_off = (FLAGS_spdlog_level == "off");

  const bool any_matched =
      want_unchanged ||
      want_trace ||
      want_debug ||
      want_info ||
      want_warn ||
      want_err ||
      want_critical ||
      want_off;
  if (!any_matched) {
    log()->critical("Unknown spdlog_level {}", FLAGS_spdlog_level);
    throw std::runtime_error("Unknown spdlog level");
  }

#ifdef HAVE_SPDLOG
  if (want_trace) {
    log()->set_level(spdlog::level::trace);
  } else if (want_debug) {
    log()->set_level(spdlog::level::debug);
  } else if (want_info) {
    log()->set_level(spdlog::level::info);
  } else if (want_warn) {
    log()->set_level(spdlog::level::warn);
  } else if (want_err) {
    log()->set_level(spdlog::level::err);
  } else if (want_critical) {
    log()->set_level(spdlog::level::critical);
  } else if (want_off) {
    log()->set_level(spdlog::level::off);
  }
#endif
}

}  // namespace logging
}  // namespace drake
