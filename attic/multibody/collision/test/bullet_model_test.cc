#include "drake/multibody/collision/bullet_model.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/shapes/geometry.h"

namespace drake {
namespace multibody {
namespace collision {
namespace {
GTEST_TEST(BulletModelTest, newBulletBoxShapeTest) {
  // Verifies that BulletModel::newBulletBoxShape() returns a pointer to a
  // btConvexHullShape instance that satisfies the following criteria:
  //    - contains 8 points
  //    - the coordinates of those points are bit-wise equal to the permutations
  //      of the positive and negative half-extents of the box in a specified
  //      order.
  const Vector3<double> half_extents{0.01, 0.04, 0.09};
  // clang-format off
  std::vector<Vector3<double>> expected_vertices{
      { half_extents(0),  half_extents(1),  half_extents(2)},
      {-half_extents(0),  half_extents(1),  half_extents(2)},
      { half_extents(0), -half_extents(1),  half_extents(2)},
      {-half_extents(0), -half_extents(1),  half_extents(2)},
      { half_extents(0),  half_extents(1), -half_extents(2)},
      {-half_extents(0),  half_extents(1), -half_extents(2)},
      { half_extents(0), -half_extents(1), -half_extents(2)},
      {-half_extents(0), -half_extents(1), -half_extents(2)},
  };
  // clang-format on
  const int expected_num_vertices = static_cast<int>(expected_vertices.size());
  std::unique_ptr<btCollisionShape> bt_shape =
      BulletModel::newBulletBoxShape(DrakeShapes::Box{2 * half_extents}, true);
  const auto bt_convex_hull_shape =
      dynamic_cast<btConvexHullShape*>(bt_shape.get());

  // Lock down implementation as btConvexHullShape as we'll need that to check
  // the vertices below.
  ASSERT_NE(bt_convex_hull_shape, nullptr);

  // Check the number of vertices.
  ASSERT_EQ(bt_convex_hull_shape->getNumVertices(), expected_num_vertices);

  // Check that the vertices have the expected values. We intentionally use a
  // tolerance of zero here because the results of queries on BulletModel may be
  // used in solving nonlinear optimization problems, where sub-epsilon
  // differences can lead to different results. We use EXPECT_NEAR rather than
  // EXPECT_EQ because EXPECT_NEAR's error message is more informative in this
  // case.
  for (int i = 0; i < expected_num_vertices; ++i) {
    btVector3 bullet_vertex;
    bt_convex_hull_shape->getVertex(i, bullet_vertex);
    EXPECT_NEAR(bullet_vertex.x(), expected_vertices[i].x(), 0.0);
    EXPECT_NEAR(bullet_vertex.y(), expected_vertices[i].y(), 0.0);
    EXPECT_NEAR(bullet_vertex.z(), expected_vertices[i].z(), 0.0);
  }
}
}  // namespace
}  // namespace collision
}  // namespace multibody
}  // namespace drake
