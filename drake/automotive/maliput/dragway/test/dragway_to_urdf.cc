/**
 @file Instantiates a dragway with a user-specified number of lanes and outputs
 a URDF model of it.
 **/
#include <gflags/gflags.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_int32(num_lanes, 2, "The number of lanes in the dragway.");
DEFINE_double(length, 10, "The length of the dragway in meters.");
// By default, each lane is 3.7m (12 feet) wide, which is the standard used by
// the U.S. interstate highway system.
DEFINE_double(lane_width, 3.7, "The width of each lane.");
// By default, the shoulder width is 3 m (10 feet) wide, which is the standard
// used by the U.S. interstate highway system.
DEFINE_double(shoulder_width, 3.0, "The width of the shoulders.");
DEFINE_string(dirpath, ".",
    "The directory path to where the URDF and OBJ files are saved");

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

  const std::string fileroot = "dragway";
  utility::ObjFeatures features;

  utility::GenerateUrdfFile(&road_geometry, FLAGS_dirpath, fileroot, features);
  return 0;
}

}  // namespace
}  // namespace dragway
}  // namespace maliput
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::maliput::dragway::exec(argc, argv);
}
