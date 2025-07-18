#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/planning/dev/voxel_collision_map_internal.h"


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
  const RigidTransformd X_PG (
    math::RollPitchYawd{M_PI / 6, M_PI / 6., 0.}, {0., 0., 0.5});  // Translation: z = 0.5
  const Vector3d grid_dimensions(1.0, 1.0, 1.0);  // 1m cube
  const double grid_resolution = 0.1;  // 10cm voxels
  const float default_occupancy = 0.5;  // Unknown by default

  planning::VoxelCollisionMap voxel_collision_map(
      parent_frame, X_PG, grid_dimensions, grid_resolution, default_occupancy);

  // Visualize the initial voxel grid
  meshcat->SetObject("voxel_grid", voxel_collision_map); // Semi-transparent black for unknown
  meshcat->Flush();

  std::cout << "\nOpen your browser to the following URL:\n\n"
            << meshcat->web_url() << "\n\n";

  std::cout << "You should see a voxelized grid visualization.\n"
            << "The grid is currently empty (all voxels unknown).\n";
  MaybePauseForUser();

  // Get mutable access to the internal collision map.
  voxelized_geometry_tools::CollisionMap& grid =
      planning::internal::GetMutableInternalCollisionMap(voxel_collision_map);

  // Set some voxels to occupied (1.0) and free (0.0).
  // Create a pattern - make bottom half are occupied and top half are free.
  for (int64_t data_index = 0; data_index < grid.GetTotalCells(); data_index++) {
    auto& cell = grid.GetDataIndexMutable(data_index);
    const auto grid_index = grid.DataIndexToGridIndex(data_index);
    const int64_t z = grid_index.Z();
    if (z < grid.GetNumZCells() / 3) {
      cell.Occupancy() = 1.0f;  // Occupied
    }
    else if (z < 2 * grid.GetNumZCells() / 3) {
      cell.Occupancy() = 0.0f;  // Free
    }
  }

  // Visualize the updated voxel grid.
  meshcat->SetObject("voxel_grid", voxel_collision_map);
  meshcat->Flush();

  std::cout << "The voxel grid has been updated.\n"
            << "Bottom half should be occupied (red) and top half free (invisible).\n";
  MaybePauseForUser();

  meshcat->Delete("voxel_grid");
  std::cout << "Voxel grid deleted. Everything should have disappeared.\n";

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
