#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/planning/dev/voxel_collision_map.h"

/* To test, you must manually run
`bazel run //geometry:meshcat_voxel_grid_manual_test`,
then follow the instructions on your console. */

namespace drake {
namespace geometry {
namespace {

using common::MaybePauseForUser;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

namespace fs = std::filesystem;

int do_main() {
  auto meshcat = std::make_shared<Meshcat>();

  // Create a simple voxel collision map
  const std::string parent_frame = "world";
  const RigidTransformd X_PG;  // Identity transform
  const Vector3d grid_dimensions(1.0, 1.0, 1.0);  // 1m cube
  const double grid_resolution = 0.1;  // 10cm voxels
  const float default_occupancy = 0.5;  // Unknown by default

  planning::VoxelCollisionMap voxel_map(
      parent_frame, X_PG, grid_dimensions, grid_resolution, default_occupancy);

  // Visualize the voxel grid
  meshcat->SetObject("voxel_grid", voxel_map,
                     Rgba(1.0, 0.0, 0.0, 1.0),  // Red for occupied
                     Rgba(0.0, 0.0, 0.0, 0.5)); // Semi-transparent black for unknown
  meshcat->Flush();

  std::cout << "\nOpen your browser to the following URL:\n\n"
            << meshcat->web_url() << "\n\n";

  std::cout << "You should see a voxelized grid visualization.\n"
            << "The grid is currently empty (all voxels unknown).\n";
  MaybePauseForUser();

  meshcat->Delete("voxel_grid");
  std::cout << "The voxel grid has been deleted.\n";

  MaybePauseForUser();

  return 0;
}

}  // namespace
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  // This enables ":add_text_logging_gflags" to control the spdlog level.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::do_main();
}
