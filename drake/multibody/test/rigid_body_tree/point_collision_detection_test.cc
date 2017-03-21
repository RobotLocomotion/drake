/* clang-format off */
#include "drake/multibody/rigid_body_tree.h"
/* clang-format on */

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"

// This is a port of a matlab tests.  It constructs a state where two
// "floating bricks" are in space (symmetrically placed across the x-y plane.)
// A set of four points is collided against the rigid body tree and the
// results (distances, nearest points, etc.) is evaluated for correctness.
//
// This omits the timed stress testing of the original matlab test and only
// considers the deterministic result of testing points against a posed
// rigid body tree.
// TODO(SeanCurtis-TRI): Kill this matlab tombstone by fully analyzing the test
// and confirming what it's specific purpose is and confirming that it includes
// only what is strictly necessary to achieve that purpose; requires code
// archaeology.

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_tree {
namespace {

using drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

GTEST_TEST(PointCollisionDetection, PointsWithMultiBodies) {
  RigidBodyTree<double> tree;
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/FallingBrick.urdf",
      drake::multibody::joints::kRollPitchYaw, &tree);
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/FallingBrickContactPoints.urdf",
      drake::multibody::joints::kRollPitchYaw, &tree);

  Eigen::Matrix<double, 24, 1> state;
  state << 0, 0.5, 1, 0, 0, 0,  // x_0, rpy_0
      0, -0.5, 1, 0, 0, 0,      // x_1, rpy_1
      0, 0, 0, 0, 0, 0,         // x_dot_0, omega_0
      0, 0, 0, 0, 0, 0;         // x_dot_1, omega_1

  VectorXd q = state.topRows(12);
  VectorXd v = state.bottomRows(12);
  auto kinematics_cache = tree.doKinematics(q, v);

  const int kPointCount = 4;
  Eigen::RowVectorXd cx(kPointCount), cy(kPointCount), cz(kPointCount);

  // The points to use in the distance query.
  cx << 0, 0, 0, 1;
  cy << 0.2499, 0, -0.25, -0.25;
  cz << 1, 1, 1, 1.75;
  Matrix3Xd points;
  points.resize(Eigen::NoChange, kPointCount);
  points << cx, cy, cz;

  VectorXd phi;
  Matrix3Xd normal, x, body_x;
  std::vector<int> body_idx;
  tree.collisionDetectFromPoints(kinematics_cache, points, phi, normal, x,
                                 body_x, body_idx, false);

  // Validates results.
  // Bullet can't promise any better than 1e-9 accuracy in this calculation.
  // TODO(SeanCurtis-TRI): When we change the point-element query, we can
  // increase the expected accuracy.
  double kThreshold = 9e-9;

  // When we parse spheres from sdf/urdf files, we enforce a minimum radius.
  // In this test, the final query point maps to such a sphere.  Thus the
  // distance/nearest point isn't computed relative to the sphere center, but
  // offset by this minimum radius.  This term allows us to account for it
  // in assessing correctness, and will be automatically flagged when that
  // value is removed by failing to compile.
  const double kMinSphereError = DrakeShapes::MIN_RADIUS;

  Vector4d expected_phi(0.0001, 0.25, 0.5, 0.25 - kMinSphereError);
  EXPECT_TRUE(CompareMatrices(phi, expected_phi, kThreshold,
                              MatrixCompareType::absolute));

  std::vector<int> expected_body_idx = {1, 1, 1, 2};
  EXPECT_EQ(body_idx.size(), expected_body_idx.size());
  for (size_t i = 0; i < body_idx.size(); ++i) {
    EXPECT_EQ(body_idx[i], expected_body_idx[i]);
  }

  Matrix3Xd expected_points;
  cx << 0, 0, 0, 1;
  cy << 0.25, 0.25, 0.25, -0.25;
  cz << 1, 1, 1, 1.5 + kMinSphereError;
  expected_points.resize(Eigen::NoChange, kPointCount);
  expected_points << cx, cy, cz;
  EXPECT_TRUE(CompareMatrices(x, expected_points, kThreshold,
                              MatrixCompareType::absolute));

  Matrix3Xd expected_normals;
  cx << 0, 0, 0, 0;
  cy << -1, -1, -1, 0;
  cz << 0, 0, 0, 1;
  expected_normals.resize(Eigen::NoChange, kPointCount);
  expected_normals << cx, cy, cz;
  EXPECT_TRUE(CompareMatrices(normal, expected_normals, kThreshold,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace multibody
}  // namespace drake
