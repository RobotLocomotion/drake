// This is a Drake-internal utility for use only as a direct dependency
// of executable (drake_cc_binary) targets.
//
// To add --spdlog_level to the gflags command line of any `drake_cc_binary`,
// add "//common:add_text_logging_gflags" to the `deps` attribute in its BUILD.

#include <string>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

// Declare the --spdlog_level gflags option.
DEFINE_string(spdlog_level, drake::logging::kSetLogLevelUnchanged,
              drake::logging::kSetLogLevelHelpMessage);

// Validate --spdlog_level and update Drake's configuration to match the flag.
namespace {
bool ValidateSpdlogLevel(const char* name, const std::string& value) {
  drake::unused(name);
  drake::logging::set_log_level(value);
  return true;
}
}  // namespace
DEFINE_validator(spdlog_level, &ValidateSpdlogLevel);
