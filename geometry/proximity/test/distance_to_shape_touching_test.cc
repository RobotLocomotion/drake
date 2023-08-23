#include "drake/geometry/proximity/distance_to_shape_touching.h"

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

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

GTEST_TEST(PointInBox, FaceNormalIfStrictlyOnFace) {
  // The box spans [-1,1]x[-2,2]x[-4,4].
  const fcl::Boxd box_B(2, 4, 8);

  // Test some representative cases but not all possible cases.
  // This point is strictly on -X face of the box.
  EXPECT_EQ(FaceNormalIfStrictlyOnFace(Vector3d(-1, 0.2, 0.3), box_B),
            -Vector3d::UnitX());
  // This point is strictly on +Y face of the box.
  EXPECT_EQ(FaceNormalIfStrictlyOnFace(Vector3d(0.1, 2, -0.3), box_B),
            Vector3d::UnitY());
  // This point is strictly on +Z face of the box.
  EXPECT_EQ(FaceNormalIfStrictlyOnFace(Vector3d(0.1, -0.2, 4), box_B),
            Vector3d::UnitZ());
  // This point is in the interior of the box.
  EXPECT_TRUE(FaceNormalIfStrictlyOnFace(Vector3d(0.1, -0.2, 0.3), box_B)
                  .array().isNaN().any());
  // This point is outside the box.
  EXPECT_TRUE(FaceNormalIfStrictlyOnFace(Vector3d(10, 10, 10), box_B)
                  .array().isNaN().any());

  // Test special cases for a point on an edge or a vertex of the box.
  // This point is on an edge of the box.
  EXPECT_TRUE(FaceNormalIfStrictlyOnFace(Vector3d(0.01, -2, 4), box_B)
                  .array().isNaN().any());
  // This point is at a vertex of the box.
  EXPECT_TRUE(FaceNormalIfStrictlyOnFace(Vector3d(1, -2, 4), box_B)
                  .array().isNaN().any());
}

GTEST_TEST(PointInBox, AxisIndexIfStrictlyOnEdge) {
  // The box spans [-1,1]x[-2,2]x[-4,4].
  const fcl::Boxd box_B(2, 4, 8);

  // Test some representative cases but not all possible cases.
  // This point is on an edge parallel to X-axis of the box.
  EXPECT_EQ(AxisIndexIfStrictlyOnEdge(Vector3d(0.1, 2, -4), box_B), 0);
  // This point is on an edge parallel to Y-axis of the box.
  EXPECT_EQ(AxisIndexIfStrictlyOnEdge(Vector3d(-1, 0.1, 4), box_B), 1);
  // This point is on an edge parallel to Z-axis of the box.
  EXPECT_EQ(AxisIndexIfStrictlyOnEdge(Vector3d(1, -2, 0.1), box_B), 2);
  // This point is in the interior of the box.
  EXPECT_EQ(AxisIndexIfStrictlyOnEdge(Vector3d(0.1, -0.2, 0.3), box_B), -1);
  // This point is outside the box.
  EXPECT_EQ(AxisIndexIfStrictlyOnEdge(Vector3d(10, 10, 10), box_B), -1);

  // Test a special case for a point at a vertex of the box.
  EXPECT_EQ(AxisIndexIfStrictlyOnEdge(Vector3d(-1, -2, -4), box_B), -1);
}

GTEST_TEST(PointInBox, IsAtVertex) {
  // The box spans [-1,1]x[-2,2]x[-4,4].
  const fcl::Boxd box_B(2, 4, 8);

  EXPECT_TRUE(IsAtVertex(Vector3d(1, 2, 4), box_B));
  // The point is in the middle of an edge.
  EXPECT_FALSE(IsAtVertex(Vector3d(0, 2, 4), box_B));
  // The point is at the center of a face.
  EXPECT_FALSE(IsAtVertex(Vector3d(0, 0, 4), box_B));
  // The point is at the center of the box.
  EXPECT_FALSE(IsAtVertex(Vector3d(0, 0, 0), box_B));
  // The point is outside the box.
  EXPECT_FALSE(IsAtVertex(Vector3d(10, 10, 10), box_B));
}

GTEST_TEST(BoxBox, ProjectedMinMax) {
  // This box spans [-2,2]x[-3,3]x[-6,6]. Half the length of its diagonal
  // is 7 because 2² + 3² + 6² = 4 + 9 + 36 = 49 = 7². We will use this fact
  // in the test later.
  const fcl::Boxd box_B(4, 6, 12);
  // Use the unit vector along the diagonal of the box.
  const Vector3d unit_vector_W = Vector3d(2, 3, 6).normalized();

  // To simplify the test, first we fix box_B's frame to World frame.
  {
    const RigidTransformd X_WB = RigidTransformd::Identity();
    double min_value, max_value;
    std::tie(min_value, max_value) =
        ProjectedMinMax(box_B, X_WB, unit_vector_W);
    // The min_value and max_value corresponds to half the length of the
    // diagonal of the box.
    const double kEps = 1e-14;
    EXPECT_NEAR(min_value, -7, kEps);
    EXPECT_NEAR(max_value, 7, kEps);
  }

  // Now verify that the pose X_WB of the box can take effect.
  {
    const RigidTransformd X_WB(Vector3d(2, 3, 6));
    double min_value, max_value;
    std::tie(min_value, max_value) =
        ProjectedMinMax(box_B, X_WB, unit_vector_W);
    // The min_value and max_value are shifted by X_WB.
    const double kEps = 1e-14;
    EXPECT_NEAR(min_value, 0, kEps);
    EXPECT_NEAR(max_value, 14, kEps);
  }
}

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
GTEST_TEST(BoxBox, MakeSeparatingVector) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  const fcl::Boxd box_A(1, 1, 1);
  const RigidTransformd X_WB(Vector3d(2, 0, 0));
  const fcl::Boxd box_B(3.0, 3.0, 3.0);

  EXPECT_EQ(
      MakeSeparatingVector(box_A, box_B, X_WA, X_WB, {Vector3d(0.1, 0, 0)}),
      -Vector3d::UnitX());
  EXPECT_EQ(
      MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d(0.1, 0, 0)}),
      Vector3d::UnitX());
  EXPECT_TRUE(
      MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d(0, 1, 1)})
          .array().isNaN().any());
  EXPECT_TRUE(
      MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d::Zero()})
          .array().isNaN().any());
  EXPECT_TRUE(
      MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {}).array().isNaN().any());
}

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
