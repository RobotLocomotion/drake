#include <chrono>
#include <iostream>

#include <gflags/gflags.h>

#include "drake/perception/point_cloud.h"

// TODO(jwnimmer-tri) Port this program to use googlebench.

DEFINE_int32(size, 1000000, "number of points in the cloud");

namespace drake {
namespace perception {
namespace {
int DoMain() {
  std::srand(5432);
  auto pc_xyz = PointCloud(FLAGS_size, pc_flags::kXYZs);
  pc_xyz.mutable_xyzs().setRandom();

  auto pc_maximal = PointCloud(FLAGS_size, pc_flags::kXYZs | pc_flags::kRGBs |
                                               pc_flags::kNormals |
                                               pc_flags::kDescriptorCurvature);

  // Make the spatial coordinates the same to avoid time differences due to
  // spatial hashing.
  pc_maximal.mutable_xyzs() = pc_xyz.xyzs();
  pc_maximal.mutable_rgbs().setRandom();
  pc_maximal.mutable_normals().setRandom();
  pc_maximal.mutable_descriptors().setRandom();

  auto start = std::chrono::high_resolution_clock::now();
  auto pc_down = pc_xyz.VoxelizedDownSample(0.02, false);
  auto post_xyz = std::chrono::high_resolution_clock::now();
  auto pc_down_max = pc_maximal.VoxelizedDownSample(0.02, false);
  auto post_max = std::chrono::high_resolution_clock::now();

  double duration_xyz =
      static_cast<std::chrono::duration<double>>(post_xyz - start).count();
  double duration_max =
      static_cast<std::chrono::duration<double>>(post_max - start).count();

  std::cout << "xyz only time " << duration_xyz << std::endl
            << "maximal fields time " << duration_max << std::endl;
  return 0;
}
}  // namespace
}  // namespace perception
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::perception::DoMain();
}
