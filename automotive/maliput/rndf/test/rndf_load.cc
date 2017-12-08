// Attempts to load a RNDF file as input and build a RNDF road geometry.
#include <iostream>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "drake/automotive/maliput/rndf/loader.h"
#include "drake/common/text_logging.h"

namespace rndf = drake::maliput::rndf;

DEFINE_string(rndf_file, "", "RNDF input file defining a RNDF road geometry");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_rndf_file.empty()) {
    drake::log()->error("No input file!");
    return 1;
  }
  drake::log()->info("Loading '{}'.", FLAGS_rndf_file);
  const auto road_geometry = rndf::LoadFile(FLAGS_rndf_file);
  const std::vector<std::string> failures = road_geometry->CheckInvariants();

  if (!failures.empty()) {
    for (const auto& f : failures) {
      drake::log()->error(f);
    }
    return 1;
  }

  return 0;
}
