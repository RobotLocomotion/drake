/**
 @file Instantiates a dragway with a user-specified number of lanes and outputs
 a URDF model of it.
 **/

#include <gflags/gflags.h>
#include "spruce.hh"

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_int32(num_lanes, 2, "The number of lanes.");
DEFINE_double(length, 10, "The length of the dragway in meters.");
// By default, each lane is 3.7m (12 feet) wide, which is the standard used by
// the U.S. interstate highway system.
DEFINE_double(lane_width, 3.7, "The width of each lane in meters.");
// By default, the shoulder width is 3 m (10 feet) wide, which is the standard
// used by the U.S. interstate highway system.
DEFINE_double(shoulder_width, 3.0,
    "The width of the shoulders in meters. Both shoulders have the same "
    "width.");
DEFINE_string(dirpath, ".",
    "The path to where the URDF and OBJ files should be saved. If this path "
    " does not exist, it is created.");
DEFINE_string(file_name_root, "dragway",
    "The root name of the files to create. For example, if the value of this "
    "parameter is \"foo\", the following files will be created: \"foo.urdf\", "
    "\"foo.obj\", and \"foo.mtl\". These files will be placed in the path "
    "specified by parameter 'dirpath'.");

namespace drake {
namespace maliput {
namespace dragway {
namespace {

int exec(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  RoadGeometry road_geometry(
      {"Dragway with " + std::to_string(FLAGS_num_lanes) + " lanes."},
      FLAGS_num_lanes,
      FLAGS_length,
      FLAGS_lane_width,
      FLAGS_shoulder_width);

  utility::ObjFeatures features;

  // Creates the destination directory if it does not already exist.
  spruce::path directory;
  directory.setStr(FLAGS_dirpath);
  if (!directory.exists()) {
    spruce::dir::mkdirAll(directory);
  }
  DRAKE_DEMAND(directory.exists());

  // The following is necessary for users to know where to find the resulting
  // files when this program is executed in a sandbox. This occurs, for example
  // when using `bazel run //drake/automotive/maliput/dragway:dragway_to_urdf`.
  spruce::path my_path;
  my_path.setAsCurrent();

  drake::log()->info("Creating Dragway URDF in {}.", my_path.getStr());
  utility::GenerateUrdfFile(&road_geometry, directory.getStr(),
      FLAGS_file_name_root, features);
  return 0;
}

}  // namespace
}  // namespace dragway
}  // namespace maliput
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::maliput::dragway::exec(argc, argv);
}
