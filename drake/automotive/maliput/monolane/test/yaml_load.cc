/// @file yaml_load.cc
///
/// Attempts to load a yaml file as input and build a monolane road geometry.
///
#include <iostream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/loader.h"
#include "drake/common/text_logging.h"

namespace mono = drake::maliput::monolane;

DEFINE_string(yaml_file, "",
              "yaml input file defining a monolane road geometry");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_yaml_file.empty()) {
    drake::log()->error("No input file!");
    return 1;
  }
  drake::log()->info("Loading '{}'.", FLAGS_yaml_file);
  auto rg = mono::LoadFile(FLAGS_yaml_file);
  const std::vector<std::string> failures = rg->CheckInvariants();

  if (!failures.empty()) {
    for (const auto& f : failures) {
      drake::log()->error(f);
    }
    return 1;
  }

  return 0;
}
