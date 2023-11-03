#include <chrono>
#include <iostream>
#include "drake/perception/point_cloud.h"

namespace drake::perception {

int do_main() {
  std::srand(1234);
  auto pc_xyz = PointCloud(1000000, pc_flags::kXYZs);
  pc_xyz.mutable_xyzs().setRandom();
  auto pc_maximal = PointCloud(1000000, pc_flags::kXYZs | pc_flags::kRGBs |
      pc_flags::kNormals | pc_flags::kDescriptorCurvature);
  pc_maximal.mutable_xyzs() = pc_xyz.xyzs();
  pc_maximal.mutable_rgbs().setRandom();
  pc_maximal.mutable_normals().setRandom();
  pc_maximal.mutable_descriptors().setRandom();

  auto start = std::chrono::high_resolution_clock::now();
  auto pc_down = pc_xyz.VoxelizedDownSample(0.02, false);
  auto post_xyz = std::chrono::high_resolution_clock::now();
  auto pc_down_max = pc_maximal.VoxelizedDownSample(0.02, false);
  auto post_max = std::chrono::high_resolution_clock::now();

  double duration_xyz = static_cast<std::chrono::duration<double>>(
      post_xyz - start
    ).count();

  double duration_max = static_cast<std::chrono::duration<double>>(
      post_max - start
  ).count();

  std::cout << "xyz only time " << duration_xyz <<
                " s\nmaximal fields time " << duration_max << std::endl;
  return 0;
}
}

int main(int argc, char *argv[]) {
  drake::perception::do_main();
}