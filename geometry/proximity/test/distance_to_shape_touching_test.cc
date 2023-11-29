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
using std::make_shared;

// Tests CalcGradientWhenTouching() when a sphere touches another object. Use
// an ellipsoid as a representative case. The other cases (convex mesh and
// general mesh) are harder to set up, but they go through the same code
// path in CalcGradientWhenTouching().
GTEST_TEST(CalcGradientWhenTouching, SphereTouch) {
  // A sphere touches an ellipsoid at World's origin.
  const double radius = 2;
  const RigidTransformd X_WA(Vector3d(-radius, 0, 0));
  const fcl::CollisionObjectd a(make_shared<fcl::Sphered>(radius),
                                X_WA.rotation().matrix(), X_WA.translation());
  const double rx = 2;
  const RigidTransformd X_WB(Vector3d(rx, 0, 0));
  const fcl::CollisionObjectd b(make_shared<fcl::Ellipsoidd>(rx, 3, 4),
                                X_WB.rotation().matrix(), X_WB.translation());
  const Vector3d p_ACa(radius, 0, 0);
  const Vector3d p_BCb(-rx, 0, 0);
  ASSERT_EQ(X_WA * p_ACa, Vector3d::Zero());
  ASSERT_EQ(X_WB * p_BCb, Vector3d::Zero());

  EXPECT_EQ(CalcGradientWhenTouching(a, X_WA, b, X_WB, p_ACa, p_BCb),
            -Vector3d::UnitX());
  // Switching `a` and `b` flips the gradient.
  EXPECT_EQ(CalcGradientWhenTouching(b, X_WB, a, X_WA, p_BCb, p_ACa),
            Vector3d::UnitX());
}

// This is just a simple test for CalcGradientWhenTouching() when a box touches
// a box. There are more comprehensive test cases at the level of
// BoxBoxGradient(), which is a subroutine of CalcGradientWhenTouching().
GTEST_TEST(CalcGradientWhenTouching, box_touches_box) {
  // Two boxes touch at their faces passing World's origin.
  const double width = 2;
  const RigidTransformd X_WA(Vector3d(-width / 2, 0, 0));
  const fcl::CollisionObjectd box_A(make_shared<Boxd>(width, 3, 4),
                                    X_WA.rotation().matrix(),
                                    X_WA.translation());
  const RigidTransformd X_WB(Vector3d(width / 2, 0, 0));
  const fcl::CollisionObjectd box_B(make_shared<Boxd>(width, 5, 6),
                                    X_WB.rotation().matrix(),
                                    X_WB.translation());
  const Vector3d p_ACa(width / 2, 0, 0);
  const Vector3d p_BCb(-width / 2, 0, 0);
  ASSERT_EQ(X_WA * p_ACa, Vector3d::Zero());
  ASSERT_EQ(X_WB * p_BCb, Vector3d::Zero());

  EXPECT_EQ(CalcGradientWhenTouching(box_A, X_WA, box_B, X_WB, p_ACa, p_BCb),
            -Vector3d::UnitX());
}

void PointOnBoxHelperTester(const fcl::Boxd box_B, const Vector3d p_BCb,
                            int expected_sum,
                            std::optional<int> expected_axis_index,
                            std::optional<const Vector3d> expected_normal) {
  const Vector3d v_B = PointOnBoxHelper(p_BCb, box_B);
  double s_B = v_B.sum();

  // Check the expected encoding sum.
  EXPECT_EQ(s_B, expected_sum);

  // If strictly on a face, check the expected normal.
  if (s_B == 1) {
    const Vector3d normal_B = (v_B.dot(p_BCb) > 0 ? v_B : -v_B);
    ASSERT_TRUE(expected_normal.has_value());
    EXPECT_EQ(normal_B, expected_normal.value());
  }

  // If strictly on an edge, check the expected axis index.
  if (s_B == 2) {
    const int axis_index_B = (Vector3d::Ones() - v_B).dot(Vector3d(0, 1, 2));
    ASSERT_TRUE(expected_axis_index.has_value());
    EXPECT_EQ(axis_index_B, expected_axis_index.value());
  }
}

GTEST_TEST(PointInBox, PointOnBoxHelper) {
  // The box spans [-1,1]x[-2,2]x[-4,4].
  const fcl::Boxd box_B(2, 4, 8);

  // This point is in the interior of the box.
  PointOnBoxHelperTester(box_B, Vector3d(0.01, -0.02, 0.03), 0, {}, {});
  // This point is outside the box.
  PointOnBoxHelperTester(box_B, Vector3d(10, 10, 10), 0, {}, {});

  // This point is strictly on -X face of the box.
  PointOnBoxHelperTester(box_B, Vector3d(-1, 0.02, 0.03), 1, {},
                         -Vector3d::UnitX());
  // This point is strictly on +Y face of the box.
  PointOnBoxHelperTester(box_B, Vector3d(0.01, 2, -0.03), 1, {},
                         Vector3d::UnitY());
  // This point is strictly on -Z face of the box.
  PointOnBoxHelperTester(box_B, Vector3d(0.01, -0.02, -4), 1, {},
                         -Vector3d::UnitZ());

  // This point is on an edge parallel to X-axis of the box.
  PointOnBoxHelperTester(box_B, Vector3d(0.1, 2, -4), 2, 0, {});
  // This point is on an edge parallel to Y-axis of the box.
  PointOnBoxHelperTester(box_B, Vector3d(-1, 0.1, 4), 2, 1, {});
  // This point is on an edge parallel to Z-axis of the box.
  PointOnBoxHelperTester(box_B, Vector3d(1, -2, 0.1), 2, 2, {});

  // The point is at a vertex of the box.
  PointOnBoxHelperTester(box_B, Vector3d(1, 2, 4), 3, {}, {});
  // The point is at a vertex of the box.
  PointOnBoxHelperTester(box_B, Vector3d(-1, 2, -4), 3, {}, {});
}

GTEST_TEST(SeparatingAxis, ProjectedMinMax) {
  // This box spans [-2,2]x[-3,3]x[-6,6]. Half the length of its diagonal
  // is 7 because 2² + 3² + 6² = 4 + 9 + 36 = 49 = 7². We will use this fact
  // in the test later.
  const fcl::Boxd box_B(4, 6, 12);
  // Use the unit vector along the diagonal of the box.
  const Vector3d unit_vector_W = Vector3d(2, 3, 6).normalized();

  // First we fix box_B's frame to World frame.
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

  // Confirm that X_WB can change the min_value and max_value by moving
  // the box along half its diagonal vector.
  {
    const RigidTransformd X_WB(Vector3d(2, 3, 6));
    double min_value, max_value;
    std::tie(min_value, max_value) =
        ProjectedMinMax(box_B, X_WB, unit_vector_W);
    // The min_value and max_value change from [-7,7] to [0,14] because the
    // box is shifted by half the diagonal.
    const double kEps = 1e-14;
    EXPECT_NEAR(min_value, 0, kEps);
    EXPECT_NEAR(max_value, 14, kEps);
  }
}

// Tests MakeSeparatingVector() on two touching boxes with various candidate
// directions for the separating axis.
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
GTEST_TEST(SeparatingAxis, MakeSeparatingVector) {
  // Fix box_A at World's origin.
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-0.5,0.5]x[-0.5,0.5]x[-0.5,0.5].
  const fcl::Boxd box_A(1, 1, 1);
  // box_B spans [-1.5,1.5]x[-1.5,1.5]x[-1.5,1.5].
  const fcl::Boxd box_B(3, 3, 3);
  // Place box_B so its face passes through a face of box_A.
  const RigidTransformd X_WB(Vector3d(2, 0, 0));

  // Any vector parallel to World's X axis can make the separating axis. It
  // doesn't have to be a unit vector.
  EXPECT_EQ(
      MakeSeparatingVector(box_A, box_B, X_WA, X_WB, {Vector3d(0.1, 0, 0)}),
      -Vector3d::UnitX());
  // Switching A and B flips the separating vector.
  EXPECT_EQ(
      MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d(0.1, 0, 0)}),
      Vector3d::UnitX());
  // Non-separating direction gets NaN.
  EXPECT_TRUE(
      MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d(0, 1, 1)})
          .array()
          .isNaN()
          .any());
  // Zero vector gets NaN.
  EXPECT_TRUE(MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d::Zero()})
                  .array()
                  .isNaN()
                  .any());
  // An empty list of vectors gets NaN.
  EXPECT_TRUE(
      MakeSeparatingVector(box_B, box_A, X_WB, X_WA, {}).array().isNaN().any());
}

// Tests BoxBoxGradient() when two boxes touch on a face. We have seen FCL
// set a witness point at a vertex of the touching face of box_A, as shown in
// this picture.
//
//        Y
//        ↑
//        | +--------------+
//        | |              |
//     +---(*)             |               (*) Witness point
//     | A  |      B       | ---→ X
//     +----+              |
//          |              |
//          +--------------+
//
GTEST_TEST(BoxBoxGradient, Face) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-0.5,0.5]x[-0.5,0.5]x[-0.5,0.5].
  const fcl::Boxd box_A(1, 1, 1);
  // box_B spans [-1.5,1.5]x[-1.5,1.5]x[-1.5,1.5].
  const fcl::Boxd box_B(3, 3, 3);
  const RigidTransformd X_WB(Vector3d(2, 0, 0));
  // On Box A, the witness point is at a vertex.
  const Vector3d p_ACa(0.5, 0.5, 0.5);
  // On Box B, the witness point is strictly inside a face.
  const Vector3d p_BCb(-1.5, 0.5, 0.5);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb),
                      -Vector3d::UnitX(), 1e-14));
  // Switching A and B flips the gradient.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa),
                      Vector3d::UnitX(), 1e-14));
}

// Tests BoxBoxGradient() when two boxes have their edges cross each other.
// Use rotation to make the witness points at the middle of the edges.
//
//        Y   +
//        ↑  / \                            .
//        | /\  \  B                           .
//        |\  \ /
//     +---(*) /                     (*) Witness point
//     | A  |\/ --------→ X
//     +----+
//
GTEST_TEST(BoxBoxGradient, EdgeCrossEdge) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_A(2, 2, 2);
  // box_B spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_B(2, 2, 2);
  // Pose box_B so that each witness point is at the middle of an edge of
  // each box.
  const RigidTransformd X_WB(AngleAxisd(M_PI_2, Vector3d(1, 1, 0)),
                             Vector3d(2, 2, 0));
  // The witness point of box_A is in the middle of an edge parallel to Az axis.
  const Vector3d p_ACa(1, 1, 0);
  // The witness point of box_B is in the middle of an edge parallel to Bz axis.
  const Vector3d p_BCb(-1, -1, 0);
  // Confirm that they co-locate in World's frame.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb),
                      Vector3d(-1, -1, 0).normalized(), 1e-14));
  // Switching A and B flips the gradient.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa),
                      Vector3d(1, 1, 0).normalized(), 1e-14));
}

// Tests BoxBoxGradient() when two boxes have their touching edges parallel
// to each other.
//
//        Y
//        ↑
//        | +----+
//        | | B  |
//     +---(*)---+                    (*) Witness point
//     | A  |  --------→ X
//     +----+
//
GTEST_TEST(BoxBoxGradient, ParallelEdgeEdge) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_A(2, 2, 2);
  // Translate diagonally in X-Y plane
  const RigidTransformd X_WB(Vector3d(2, 2, 0));
  // box_B spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_B(2, 2, 2);
  // The witness point of box_A is in the middle of an edge parallel to Az axis.
  const Vector3d p_ACa(1, 1, 0);
  // The witness point of box_B is in the middle of an edge parallel to Bz axis.
  const Vector3d p_BCb(-1, -1, 0);
  // Confirm that they co-locate in World's frame.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb),
                      -Vector3d::UnitX()));
  // Switching A and B flips the gradient.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa),
                      Vector3d::UnitX()));
}

// Tests BoxBoxGradient() when a vertex of a box touches an edge of another box.
//
//        Y   +
//        ↑  / \         X-Y view
//        | /   \                           .
//        |\  B /
//     +---(*) /                       (*) Witness point
//     | A  |\/ --------→ X
//     +----+
//
//        Z
//        ↑
//        |              X-Z view
//        +---+--+
//        |   |  | B
//     +--+(*)+--+
//     | A  | ----------→ X
//     +----+
GTEST_TEST(BoxBoxGradient, EdgeVertex) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_A(2, 2, 2);
  using std::sqrt;
  const RigidTransformd X_WB(AngleAxisd(M_PI_4, Vector3d::UnitZ()),
                             Vector3d(1 + 1 / sqrt(2), 1 + 1 / sqrt(2), 2));
  // box_B spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_B(2, 2, 2);
  // The witness point of box_A is at the most positive +X+Y+Z vertex.
  const Vector3d p_ACa = Vector3d::Ones();
  // The witness point of box_B is in the middle of an edge parallel to By axis.
  const Vector3d p_BCb = Vector3d(-1, 0, -1);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb),
                      -Vector3d::UnitZ(), 1e-14));
  // Switching A and B flips the gradient.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa),
                      Vector3d::UnitZ(), 1e-14));
}

// Test BoxBoxGradient() when a vertex of a box touches a vertex of another
// box in such a way that a face normal is a separating axis. In this
// test case, we translate an identical box by its diagonal vector.
//
//        Y
//        ↑
//        | +----+
//        | | B  |
//     +---(*)---+                      (*) Witness point
//     | A  |  --------→ X
//     +----+
//
GTEST_TEST(BoxBoxGradient, VertexVertexUseFaceNormal) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_A(2, 2, 2);
  const RigidTransformd X_WB(Vector3d(2, 2, 2));
  // box_A spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_B(2, 2, 2);
  // The witness point of box_A is at the most positive +X+Y+Z vertex.
  const Vector3d p_ACa = Vector3d::Ones();
  // The witness point of box_B is at the most negative -X-Y-Z vertex.
  const Vector3d p_BCb = -Vector3d::Ones();
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  // All three axes X, Y, and Z are valid separating axes. The code happens to
  // pick X.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb),
                      -Vector3d::UnitX(), 1e-14));
  // Switching A and B flips the gradient.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa),
                      Vector3d::UnitX(), 1e-14));
}

// This is a special case of testing BoxBoxGradient() when a vertex of a box
// touches a vertex of another box. Two identical boxes touch each other
// on a face, but FCL can pick a witness point at a vertex of the touching face.
//
//        Y
//        ↑
//        |
//        |
//     +---(*)---+               (*) = witness point
//     | A  | B  | -----→ X
//     +----+----+
//
GTEST_TEST(BoxBoxGradient, VertexVetexUseFaceNormalSpecial) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_A(2, 2, 2);
  const RigidTransformd X_WB(Vector3d(2, 0, 0));
  // box_B spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_B(2, 2, 2);
  // Witness points are at vertices.
  const Vector3d p_ACa(1, 1, 1);
  const Vector3d p_BCb(-1, 1, 1);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  // X-axis is the only separating axis.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb),
                      -Vector3d::UnitX(), 1e-14));
  // Switching A and B flips the gradient.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa),
                      Vector3d::UnitX(), 1e-14));
}

// Tests BoxBoxGradient() when two boxes touch at their vertices in such a
// way that the separating axis is along the cross product of their edges.
GTEST_TEST(BoxBoxGradient, VertexVertexNeedCrossProduct) {
  const RigidTransformd X_WA = RigidTransformd::Identity();
  // box_A spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_A(2, 2, 2);
  // Rotation around a diagonal direction of the box preserves the two opposite
  // vertices of the box. Translation by the diagonal vector makes one of
  // such vertices touch the identical box at a vertex.
  const RigidTransformd X_WB(AngleAxisd(M_PI_4, Vector3d::Ones()),
                             Vector3d(2, 2, 2));
  // box_B spans [-1,1]x[-1,1]x[-1,1].
  const fcl::Boxd box_B(2, 2, 2);
  // Witness points are at vertices.
  const Vector3d p_ACa(1, 1, 1);
  const Vector3d p_BCb(-1, -1, -1);
  // Confirm that they co-locate.
  ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

  // A separating axis is along the cross product of Ax and Bx. There are
  // other valid directions too. The code happens to pick this one.
  const Vector3d expect_gradient =
      -X_WA.rotation().col(0).cross(X_WB.rotation().col(0)).normalized();
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb),
                      expect_gradient, 1e-14));
  // Switching A and B flips the gradient.
  EXPECT_TRUE(
      CompareMatrices(BoxBoxGradient(box_B, box_A, X_WB, X_WA, p_BCb, p_ACa),
                      -expect_gradient, 1e-14));
}

}  // namespace
}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
