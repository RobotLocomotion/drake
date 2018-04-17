#include "drake/multibody/collision/bullet_model.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/shapes/geometry.h"

namespace drake {
namespace multibody {
namespace collision {
namespace {
GTEST_TEST(BulletModelTest, newBulletBoxShapeTest) {
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
  const int expected_num_vertices = expected_vertices.size();
  std::unique_ptr<btCollisionShape> bt_shape =
      BulletModel::newBulletBoxShape(DrakeShapes::Box{2 * half_extents}, true);
  const auto bt_convex_hull_shape =
      dynamic_cast<btConvexHullShape*>(bt_shape.get());

  // Lock down implementation as btConvexHullShape as we'll need that to check
  // the vertices below.
  ASSERT_TRUE(bt_convex_hull_shape);

  // Check the number of vertices.
  ASSERT_EQ(bt_convex_hull_shape->getNumVertices(), expected_num_vertices);

  // Check that the vertices have the expected values. We are intentionally
  // comparing doubles here because the results of queries on BulletModel may be
  // used in solving nonlinear optimization problems, where epsilon differences
  // can lead to different results.
  for (int i = 0; i < expected_num_vertices; ++i) {
    btVector3 bullet_vertex;
    bt_convex_hull_shape->getVertex(i, bullet_vertex);
    EXPECT_EQ(bullet_vertex.x() - expected_vertices[i].x(), 0.);
    EXPECT_EQ(bullet_vertex.y() - expected_vertices[i].y(), 0.);
    EXPECT_EQ(bullet_vertex.z() - expected_vertices[i].z(), 0.);
  }
}
}  // namespace
}  // namespace collision
}  // namespace multibody
}  // namespace drake
