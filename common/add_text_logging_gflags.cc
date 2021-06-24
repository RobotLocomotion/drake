// This is a Drake-internal utility for use only as a direct dependency
// of executable (drake_cc_binary) targets.
//
// To add --spdlog_level and --spdlog_pattern to the gflags command line of any
// `drake_cc_binary`, add "//common:add_text_logging_gflags" to the `deps`
// attribute in its BUILD.

#include <string>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

// Declare the gflags options.
DEFINE_string(spdlog_level, drake::logging::kSetLogLevelUnchanged,
              drake::logging::kSetLogLevelHelpMessage);
DEFINE_string(spdlog_pattern, "%+", drake::logging::kSetLogPatternHelpMessage);

// Validate flags and update Drake's configuration to match their values.
namespace {
bool ValidateSpdlogLevel(const char* name, const std::string& value) {
  drake::unused(name);
  drake::logging::set_log_level(value);
  return true;
}
bool ValidateSpdlogPattern(const char* name, const std::string& value) {
  drake::unused(name);
  drake::logging::set_log_pattern(value);
  return true;
}
}  // namespace
DEFINE_validator(spdlog_level, &ValidateSpdlogLevel);
DEFINE_validator(spdlog_pattern, &ValidateSpdlogPattern);
