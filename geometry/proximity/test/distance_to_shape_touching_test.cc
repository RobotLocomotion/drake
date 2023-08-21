#include "drake/geometry/proximity/distance_to_shape_touching.h"

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

// @file Test special cases when two shapes touch each other for
// ComputeNarrowPhaseDistance() and CalcDistanceFallback() in
// distance_to_shape_callback.

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using fcl::Boxd;
using math::RigidTransformd;
using math::RollPitchYawd;

// On a face.
// On Box A, the witness point is at a vertex.
// On Box B, the witness point is strictly inside a face.
//
//        Y
//        ↑
//        | +--------------+
//        | |              |
//     +----+              |
//     | A  |      B       | ---→ X
//     +----+              |
//          |              |
//          +--------------+
//
GTEST_TEST(BoxBoxGradient, Face) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(1, 1, 1);
  const RigidTransformd X_WB(Vector3d{2.0, 0, 0});
  const fcl::Boxd box_B(3.0, 3.0, 3.0);
  const Vector3d p_ACa(0.5, 0.5, 0.5);
  const Vector3d p_BCb(-1.5, 0.5, 0.5);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  // Call (A,B)
  {
    const Vector3d nhat_BA_W =
        BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);
    EXPECT_TRUE(CompareMatrices(nhat_BA_W, -Vector3d::UnitX(), 1e-14));
  }
  // Call (B,A)
  {
    const Vector3d nhat_AB_W =
        BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa);
    EXPECT_TRUE(CompareMatrices(nhat_AB_W, Vector3d::UnitX(), 1e-14));
  }
}

// Edge-cross-edge. Use rotation to make the witness points unique at the
// middle of the edges.
//
//        Y   +
//        ↑  / \                            .
//        | /   \                           .
//        |\  B /
//     +----+  /
//     | A  |\/ --------→ X
//     +----+
//
GTEST_TEST(BoxBoxGradient, EdgeCrossEdge) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(2.0, 2.0, 2.0);
  const RigidTransformd X_WB(AngleAxisd(M_PI_2, Vector3d(1.0, 1.0, 0)),
                             Vector3d{2.0, 2.0, 0});
  const fcl::Boxd box_B(2.0, 2.0, 2.0);
  const Vector3d p_ACa(1, 1, 0);
  const Vector3d p_BCb(-1, -1, 0);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  const Vector3d nhat_BA_W =
      BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);

  EXPECT_TRUE(CompareMatrices(nhat_BA_W,
                              Vector3d(-1, -1, 0) / sqrt(2.0), 1e-14));
}

// Parallel edge-to-edge.
//
//        Y
//        ↑
//        | +----+
//        | | B  |
//     +----+----+
//     | A  |  --------→ X
//     +----+
//
GTEST_TEST(BoxBoxGradient, ParallelEdgeEdge) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(2, 2, 2);
  // Translate diagonally in X-Y
  const RigidTransformd X_WB(Vector3d(2.0, 2.0, 0.0));
  const fcl::Boxd box_B(2.0, 2.0, 2.0);
  // Witness points in the middle of the edges.
  const Vector3d p_ACa(1.0, 1.0, 0.0);
  const Vector3d p_BCb(-1.0, -1.0, 0.0);

  const Vector3d nhat_BA_W =
      BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);

  EXPECT_TRUE(CompareMatrices(nhat_BA_W, -Vector3d::UnitX()));
}

// Vertex-on-edge.
//
//        Y   +
//        ↑  / \                            .
//        | /   \                           .
//        |\  B /
//     +----+  /
//     | A  |\/ --------→ X
//     +----+
//
GTEST_TEST(BoxBoxGradient, EdgeVertex) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(2.0, 2.0, 2.0);
  using std::sqrt;
  const RigidTransformd X_WB(AngleAxisd(M_PI_4, Vector3d::UnitZ()),
                             Vector3d{1.0 + 1.0 / sqrt(2.0),
                                      1.0 + 1.0 / sqrt(2.0),
                                      2.0});
  const fcl::Boxd box_B(2.0, 2.0, 2.0);
  const Vector3d p_ACa = Vector3d::Ones();
  const Vector3d p_BCb = Vector3d(-1, 0, -1);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  // Call (A,B)
  {
    const Vector3d nhat_BA_W =
        BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);
    EXPECT_TRUE(CompareMatrices(nhat_BA_W, -Vector3d::UnitZ(), 1e-14));
  }
  // Call (B,A)
  {
    const Vector3d nhat_AB_W =
        BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa);
    EXPECT_TRUE(CompareMatrices(nhat_AB_W, Vector3d::UnitZ(), 1e-14));
  }
}

// Vertex-on-vertex. Case 1.
// Translate identical box by a diagonal vector.
//
//        Y
//        ↑
//        | +----+
//        | | B  |
//     +----+----+
//     | A  |  --------→ X
//     +----+
//
GTEST_TEST(BoxBoxGradient, VertexVertex) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(2.0, 2.0, 2.0);
  const RigidTransformd X_WB(Vector3d{2.0, 2.0, 2.0});
  const fcl::Boxd box_B(2.0, 2.0, 2.0);
  // Witness points are at vertices.
  const Vector3d p_ACa = Vector3d::Ones();
  const Vector3d p_BCb = -Vector3d::Ones();
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  // Call (A,B)
  {
    const Vector3d nhat_BA_W =
        BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);
    EXPECT_TRUE(CompareMatrices(nhat_BA_W, -Vector3d::UnitX(), 1e-14));
  }
  // Call (B,A)
  {
    const Vector3d nhat_AB_W =
        BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa);
    EXPECT_TRUE(CompareMatrices(nhat_AB_W, Vector3d::UnitX(), 1e-14));
  }
}

// Vertex-on-vertex. Case 2. Two boxes sharing a face have witness points at
// their vertices. When two identical boxes share a face, FCL can pick
// a witness point anywhere on that face including a vertex.
//
//        Y
//        ↑
//        |              * = witness point
//        |
//     +----*----+
//     | A  | B  |    ---→ X
//     +----+----+
//
GTEST_TEST(BoxBoxGradient, VertexVetexUseFaceNormal) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(2, 2, 2);
  const RigidTransformd X_WB(Vector3d(2, 0, 0));
  const fcl::Boxd box_B(2, 2, 2);
  // Witness points are at vertices.
  const Vector3d p_ACa(1, 1, 1);
  const Vector3d p_BCb(-1, 1, 1);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  const Vector3d nhat_BA_W =
      BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);

  EXPECT_TRUE(CompareMatrices(nhat_BA_W, -Vector3d::UnitX(), 1e-14));
}

// Vertex-on-vertex. Case 3. Need cross product.
GTEST_TEST(BoxBoxGradient, VertexVertexNeedCrossProduct) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(2, 2, 2);
  const RigidTransformd X_WB(AngleAxisd(M_PI_4, Vector3d::Ones()),
                             Vector3d(2, 2, 2));
  const fcl::Boxd box_B(2, 2, 2);
  // Witness points are at vertices.
  const Vector3d p_ACa(1, 1, 1);
  const Vector3d p_BCb(-1, -1, -1);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  const Vector3d nhat_BA_W =
      BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);

  EXPECT_TRUE(CompareMatrices(
      nhat_BA_W,
      -X_WA.rotation().col(0).cross(X_WB.rotation().col(0)).normalized(),
      1e-14));
}

}  // namespace
}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
