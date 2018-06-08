#pragma once

/// @file
/// This file defines gflags settings to control spdlog levels.
/// Only include this from translation units that declare a `main` function.

#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"

// Declare the --spdlog_level gflags option.
DEFINE_string(spdlog_level, "unchanged",
              "sets the spdlog output threshold; "
              "possible values are 'unchanged', 'trace', 'debug', 'info', "
                      "'warn', 'err', 'critical', 'off'");

namespace drake {
namespace logging {

inline void HandleSpdlogGflags() {
  std::shared_ptr<logging::logger> log_console = ReturnDefaultLogger();
  SetLoggerLevel(log_console, FLAGS_spdlog_level);
}

/// Creates loggers at specified logging levels. The example below sets the
/// default logger to "off", creates a logger named "logger_a" and sets
/// it to trace and creates a logger named "logger_b" and sets it to info.
/// <pre>
///  ./compiled_project --spdlog_level=off logger_a trace logger_b info
/// </pre>
inline void HandleSpdlogGflags(int argc, char *argv[]) {
  HandleSpdlogGflags();

  if (argc % 2 == 0) {
    throw std::runtime_error("Commandline argument number is odd.");
  }

  for (int k = 1; k < argc; k+=2) {
    std::string logger_name = argv[k];
    std::string logger_level = argv[k + 1];
    MakeLoggerSetLevel(logger_name, logger_level);
  }
}

}  // namespace logging
}  // namespace drake
