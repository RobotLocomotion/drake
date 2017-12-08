// Takes a RNDF file as input, builds the resulting RNDF road geometry and
// renders the road surface, saved as a WaveFront OBJ output file.
#include <gflags/gflags.h>

#include "drake/automotive/maliput/rndf/loader.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"

namespace rndf = drake::maliput::rndf;
namespace utility = drake::maliput::utility;

DEFINE_string(rndf_file, "", "RNDF input file");
DEFINE_string(obj_dir, ".", "Directory to contain rendered road surface");
DEFINE_string(obj_file, "", "Basename for output Wavefront OBJ and MTL files");
DEFINE_double(max_grid_unit, utility::ObjFeatures().max_grid_unit,
              "Maximum size of a grid unit in the rendered mesh covering the"
              " road surface");
DEFINE_double(min_grid_resolution, utility::ObjFeatures().min_grid_resolution,
              "Minimum number of grid-units in either lateral or longitudinal"
              " direction in the rendered mesh covering the road surface");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  if (FLAGS_rndf_file.empty()) {
    drake::log()->critical("No input file specified.");
    return 1;
  }
  if (FLAGS_obj_file.empty()) {
    drake::log()->critical("No output file specified.");
    return 1;
  }

  drake::log()->info("Loading road geometry.");
  const auto road_geometry = rndf::LoadFile(FLAGS_rndf_file);

  utility::ObjFeatures features;
  features.max_grid_unit = FLAGS_max_grid_unit;
  features.min_grid_resolution = FLAGS_min_grid_resolution;

  drake::log()->info("Generating OBJ.");
  utility::GenerateObjFile(road_geometry.get(), FLAGS_obj_dir, FLAGS_obj_file,
                           features);
  return 0;
}
