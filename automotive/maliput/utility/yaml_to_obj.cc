/// @file yaml_to_obj.cc
///
/// Take a yaml file as input, build the resulting multilane road geometry, and
/// render the road surface to a WaveFront OBJ output file.
#include <string>

#include <gflags/gflags.h>
#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_string(yaml_file, "",
              "yaml input file defining a multilane road geometry");
DEFINE_string(obj_dir, ".", "Directory to contain rendered road surface");
DEFINE_string(obj_file, "", "Basename for output Wavefront OBJ and MTL files");
DEFINE_double(max_grid_unit,
              drake::maliput::utility::ObjFeatures().max_grid_unit,
              "Maximum size of a grid unit in the rendered mesh covering the "
              "road surface");
DEFINE_double(min_grid_resolution,
              drake::maliput::utility::ObjFeatures().min_grid_resolution,
              "Minimum number of grid-units in either lateral or longitudinal "
              "direction in the rendered mesh covering the road surface");
DEFINE_bool(draw_elevation_bounds,
            drake::maliput::utility::ObjFeatures().draw_elevation_bounds,
            "Whether to draw the elevation bounds");
DEFINE_double(simplify_mesh_threshold,
              drake::maliput::utility::ObjFeatures().simplify_mesh_threshold,
              "Optional tolerance for mesh simplification, in meters. Make it "
              "equal to the road linear tolerance to get a mesh size reduction "
              "while keeping geometrical fidelity.");

namespace drake {
namespace maliput {
namespace utility {
namespace {

// Available maliput implementations to load.
enum class MaliputImplementation {
  kMultilane,  //< multilane implementation.
  kUnknown     //< Used when none of the implementations could be identified.
};

// Parses a file whose path is `filename` as a YAML and looks for a node called
// "maliput_multilane_builder". If it is found,
// MaliputImplementation::kMultilane is returned. Otherwise,
// MaliputImplementation::kUnknown is returned.
MaliputImplementation GetMaliputImplementation(const std::string& filename) {
  const YAML::Node yaml_file = YAML::LoadFile(filename);
  DRAKE_DEMAND(yaml_file.IsMap());
  if (yaml_file["maliput_multilane_builder"]) {
    return MaliputImplementation::kMultilane;
  }
  return MaliputImplementation::kUnknown;
}

// Generates an OBJ file from a YAML file path given as CLI argument.
int main(int argc, char* argv[]) {
  drake::log()->debug("main()");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  if (FLAGS_yaml_file.empty()) {
    drake::log()->critical("No input file specified.");
    return 1;
  }
  if (FLAGS_obj_file.empty()) {
    drake::log()->critical("No output file specified.");
    return 1;
  }

  drake::log()->info("Loading road geometry...");
  std::unique_ptr<const drake::maliput::api::RoadGeometry> rg{};
  switch (GetMaliputImplementation(FLAGS_yaml_file)) {
    case MaliputImplementation::kMultilane: {
      rg = drake::maliput::multilane::LoadFile(
          drake::maliput::multilane::BuilderFactory(), FLAGS_yaml_file);
      drake::log()->info("Loaded a multilane road geometry.");
      break;
    }
    default: {
      drake::log()->error("Unknown map.");
      DRAKE_ABORT();
      break;
    }
  }

  utility::ObjFeatures features;
  features.max_grid_unit = FLAGS_max_grid_unit;
  features.min_grid_resolution = FLAGS_min_grid_resolution;
  features.draw_elevation_bounds = FLAGS_draw_elevation_bounds;
  features.simplify_mesh_threshold = FLAGS_simplify_mesh_threshold;
  drake::log()->info("Generating OBJ.");
  GenerateObjFile(rg.get(), FLAGS_obj_dir, FLAGS_obj_file, features);

  return 0;
}

}  // namespace
}  // namespace utility
}  // namespace maliput
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::maliput::utility::main(argc, argv);
}
