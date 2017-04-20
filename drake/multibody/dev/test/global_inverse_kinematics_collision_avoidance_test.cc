#include "drake/multibody/dev/test/global_inverse_kinematics_test_util.h"

#include "drake/solvers/mosek_solver.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
TEST_F(KukaTest, CollisionAvoidanceTest) {
  // Suppose there is a box with length 0.1m, located at [0.5, 0, 0.4]
  // The goal is to reach the goal position, without colliding with the box
  Eigen::Vector3d box_size(0.1, 0.1, 0.1);
  Eigen::Vector3d box_pos(0.5, 0, 0.4);
  double margin = 0.05;

  // Now first partition the free space into 6 regions.
  std::vector<Eigen::Matrix3Xd> region_vertices(6);

  // region_vertices[0] are the vertices of the free space region in the +z
  // direction above the box.
  region_vertices[0].resize(3, 8);
  region_vertices[0].row(0) << -0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5;
  region_vertices[0].row(1) << -1, 1, -1, 1, -1, 1, -1, 1;
  region_vertices[0].row(2) << Eigen::RowVector4d::Constant(box_pos(2) + box_size(2) / 3 + margin), Eigen::RowVector4d::Constant(box_pos(2) + box_size(2) / 3 + margin + 0.6);

  // region_vertices[1] are the vertices of the free space region in the -z
  // direction below the box
  region_vertices[1].resize(3, 4);
  region_vertices[1].row(0) = region_vertices[0].row(0);
  region_vertices[1].row(1) = region_vertices[0].row(1);
  region_vertices[1].row(2) << Eigen::RowVector

}
}  // namespace
}  // namespace multibody
}  // namespace drake
