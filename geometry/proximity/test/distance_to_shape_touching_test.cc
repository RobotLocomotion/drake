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
using math::RotationMatrixd;
using std::make_shared;

// Tests CalcGradientWhenTouching() when a sphere touches another object. Use
// the Ellipsoid as indicative. Inspection of the calling code should make it
// apparent that the correctness of Sphere-X contact doesn't depend on what
// the X is, so we don't bother testing other, redundant types.
GTEST_TEST(CalcGradientWhenTouching, SphereTouch) {
  // Strengthen the test by an arbitrary frame F, so we can use simple poses
  // X_FA and X_FB of the sphere's frame A in F and the ellipsoid's frame B in
  // F in such a way that X_WA and X_WB are complex.
  const RigidTransformd X_WF(RollPitchYawd(M_PI, M_PI_2, M_PI_4),
                             Vector3d(0.5, 1, 2));
  // A sphere touches an ellipsoid at F's origin.
  const double radius = 2;
  const RigidTransformd X_FA(Vector3d(-radius, 0, 0));
  const RigidTransformd X_WA = X_WF * X_FA;
  const fcl::CollisionObjectd a(make_shared<fcl::Sphered>(radius),
                                X_WA.rotation().matrix(), X_WA.translation());
  const double rx = 2;
  const RigidTransformd X_FB(Vector3d(rx, 0, 0));
  const RigidTransformd X_WB = X_WF * X_FB;
  const fcl::CollisionObjectd b(make_shared<fcl::Ellipsoidd>(rx, 3, 4),
                                X_WB.rotation().matrix(), X_WB.translation());
  const Vector3d p_ACa(radius, 0, 0);
  const Vector3d p_BCb(-rx, 0, 0);
  ASSERT_EQ(X_FA * p_ACa, Vector3d::Zero());
  ASSERT_EQ(X_FB * p_BCb, Vector3d::Zero());

  const Vector3d expect_nhat_BA_F = -Vector3d::UnitX();
  EXPECT_EQ(CalcGradientWhenTouching(a, X_WA, b, X_WB, p_ACa, p_BCb),
            X_WF.rotation() * expect_nhat_BA_F);
  // Switching `a` and `b` flips the gradient.
  const Vector3d expect_nhat_AB_F = Vector3d::UnitX();
  EXPECT_EQ(CalcGradientWhenTouching(b, X_WB, a, X_WA, p_BCb, p_ACa),
            X_WF.rotation() * expect_nhat_AB_F);
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

void PointOnBoxSurfaceHelperTester(
    const fcl::Boxd box_B, const Vector3d p_BCb, int expected_sum,
    std::optional<int> expected_axis_index,
    std::optional<const Vector3d> expected_normal) {
  const Vector3d v_B = PointOnBoxSurfaceHelper(p_BCb, box_B);
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

GTEST_TEST(PointInBox, PointOnBoxSurfaceHelper) {
  // The box spans [-1,1]x[-2,2]x[-4,4].
  const fcl::Boxd box_B(2, 4, 8);
  {
    SCOPED_TRACE("This point is in the interior of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(0.01, -0.02, 0.03), 0, {},
                                  {});
  }
  {
    SCOPED_TRACE("This point is outside the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(10, 10, 10), 0, {}, {});
  }
  {
    SCOPED_TRACE("This point is strictly on -X face of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(-1, 0.02, 0.03), 1, {},
                                  -Vector3d::UnitX());
  }
  {
    SCOPED_TRACE("This point is strictly on +Y face of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(0.01, 2, -0.03), 1, {},
                                  Vector3d::UnitY());
  }
  {
    SCOPED_TRACE("This point is strictly on -Z face of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(0.01, -0.02, -4), 1, {},
                                  -Vector3d::UnitZ());
  }
  {
    SCOPED_TRACE("This point is on an edge parallel to X-axis of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(0.1, 2, -4), 2, 0, {});
  }
  {
    SCOPED_TRACE("This point is on an edge parallel to Y-axis of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(-1, 0.1, 4), 2, 1, {});
  }
  {
    SCOPED_TRACE("This point is on an edge parallel to Z-axis of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(1, -2, 0.1), 2, 2, {});
  }
  {
    SCOPED_TRACE("The point is at the +X+Y+Z vertex of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(1, 2, 4), 3, {}, {});
  }
  {
    SCOPED_TRACE("The point is at the -X+Y-Z vertex of the box.");
    PointOnBoxSurfaceHelperTester(box_B, Vector3d(-1, 2, -4), 3, {}, {});
  }
}

// Test corner cases of PointOnBoxSurfaceHelper() due to numerical errors.
// It is inspired by issue 21192. This test shows that we can get wrong
// classification due to imperfect numerical precisions.
GTEST_TEST(PointOnBoxSurfaceHelper, CornerCases) {
  const fcl::Boxd box_B(0.25, 0.25, 0.25);
  // See documentation of PointOnBoxSurfaceHelper() for the meaning of these
  // return encoding vector.
  const Vector3d kVertexXYZ(1, 1, 1);
  const Vector3d kEdgeXY(1, 1, 0);
  const Vector3d kFaceX(1, 0, 0);
  const Vector3d kNone(0, 0, 0);
  {
    // Position of a corner vertex of the box.
    const Vector3d p_BQ(0.125, 0.125, 0.125);

    // With perfect precisions, it is classified as a vertex of the box.
    EXPECT_EQ(PointOnBoxSurfaceHelper(p_BQ, box_B), kVertexXYZ);
  }
  {
    // Within an acceptable tolerance, it is still classified as a vertex.
    const Vector3d p_BQ(0.125 - 1e-14, 0.125 - 1e-14, 0.125 - 1e-14);
    EXPECT_EQ(PointOnBoxSurfaceHelper(p_BQ, box_B), kVertexXYZ);
  }
  {
    // However, more numerical errors can trigger wrong classification in
    // various ways.
    const Vector3d p_BQ_edge(0.125, 0.125, 0.125 - 1e-13);
    EXPECT_EQ(PointOnBoxSurfaceHelper(p_BQ_edge, box_B), kEdgeXY);
    const Vector3d p_BQ_face(0.125, 0.125 - 1e-13, 0.125 - 1e-13);
    EXPECT_EQ(PointOnBoxSurfaceHelper(p_BQ_face, box_B), kFaceX);
    const Vector3d p_BQ_none(0.125 - 1e-13, 0.125 - 1e-13, 0.125 - 1e-13);
    EXPECT_EQ(PointOnBoxSurfaceHelper(p_BQ_none, box_B), kNone);
  }
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
    EXPECT_DOUBLE_EQ(min_value, -7);
    EXPECT_DOUBLE_EQ(max_value, 7);
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
    EXPECT_DOUBLE_EQ(min_value, 0);
    EXPECT_DOUBLE_EQ(max_value, 14);
  }
}

// Tests MaybeMakeSeparatingVector() on two touching boxes with various
// candidate directions for the separating axis.
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
GTEST_TEST(SeparatingAxis, MaybeMakeSeparatingVector) {
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
  {
    const std::optional<Vector3d> nhat_BA_W = MaybeMakeSeparatingVector(
        box_A, box_B, X_WA, X_WB, {Vector3d(0.1, 0, 0)});
    EXPECT_TRUE(nhat_BA_W.has_value());
    EXPECT_EQ(nhat_BA_W.value(), -Vector3d::UnitX());
  }
  // Switching A and B flips the separating vector.
  {
    const std::optional<Vector3d> nhat_BA_W = MaybeMakeSeparatingVector(
        box_B, box_A, X_WB, X_WA, {Vector3d(0.1, 0, 0)});
    EXPECT_TRUE(nhat_BA_W.has_value());
    EXPECT_EQ(nhat_BA_W.value(), Vector3d::UnitX());
  }
  // Non-separating direction gets null.
  EXPECT_FALSE(
      MaybeMakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d(0, 1, 1)})
          .has_value());
  // Zero vector gets null.
  EXPECT_FALSE(
      MaybeMakeSeparatingVector(box_B, box_A, X_WB, X_WA, {Vector3d::Zero()})
          .has_value());
  // An empty list of vectors gets null.
  EXPECT_FALSE(
      MaybeMakeSeparatingVector(box_B, box_A, X_WB, X_WA, {}).has_value());
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

// Users provided data for this test case. See issue 21192. Visually the two
// boxes touched along an edge, but numerically they overlapped within a very
// small tolerance. Furthermore, the caller provided one of the witness points
// at the end of the touching edge.
GTEST_TEST(BoxBoxGradient, Issue21192) {
  const fcl::Boxd box_A(0.135999, 0.167092, 0.276916);
  const Vector3d half_size_A = box_A.side / 2;
  const fcl::Boxd box_B(0.132334, 0.177387, 0.137409);
  const Vector3d half_size_B = box_B.side / 2;
  const RigidTransformd X_WA(
      RotationMatrixd(RollPitchYawd(-0.9999901072383376 * M_PI,
                                    -0.472887818233649 * M_PI,
                                    -0.5000099287557301 * M_PI)),
      Vector3d(-0.01554599999994518, 0.434174978106377, 0.6778455642817045));
  const RigidTransformd X_WB(
      RotationMatrixd(RollPitchYawd(0.5000049564737601 * M_PI,
                                    0.4301511633951143 * M_PI,
                                    0.5000038803050185 * M_PI)),
      Vector3d(-2.897114691894844e-17, 0.2298401771670561, 0.5144261282155572));
  const Vector3d p_ACa(-0.06799949999996824, 0.05315817907282317,
                       0.1384579999999353);
  const Vector3d p_BCb(-0.066166999999969, 0.08869349999995851,
                       -0.06870450000000002);

  // The following sequence of assertions provides useful information about
  // the input.
  {
    // The two witness points co-locate within a tolerance.
    ASSERT_TRUE(CompareMatrices(X_WA * p_ACa, X_WB * p_BCb, 1e-14));

    // The two boxes overlap. Overlapping boxes cannot have separating vectors
    // unless we have a reasonable tolerance in the code.
    //
    // Ca is in box_B.
    const Vector3d p_BCa = X_WB.inverse() * X_WA * p_ACa;
    ASSERT_TRUE((p_BCa.cwiseAbs().array() <= half_size_B.array()).all());
    // Cb is in box_A.
    const Vector3d p_ACb = X_WA.inverse() * X_WB * p_BCb;
    ASSERT_TRUE((p_ACb.cwiseAbs().array() <= half_size_A.array()).all());
    // The first witness point p_ACa is on an edge of box_A 3cm from a vertex.
    using std::abs;
    ASSERT_NEAR(abs(p_ACa.x()), half_size_A.x(), 1e-13);
    ASSERT_NEAR(abs(abs(p_ACa.y()) - half_size_A.y()), 0.030, 3e-3);
    ASSERT_NEAR(abs(p_ACa.z()), half_size_A.z(), 1e-13);
    // The second witness point p_BCb is near a vertex of box_B.
    ASSERT_NEAR(abs(p_BCb.x()), half_size_B.x(), 1e-13);
    ASSERT_NEAR(abs(p_BCb.y()), half_size_B.y(), 1e-13);
    ASSERT_NEAR(abs(p_BCb.z()), half_size_B.z(), 1e-13);
  }

  // Confirm that our classifcation makes sense: p_ACa is classified on an edge,
  // and p_BCb is classified on a vertex. (However, visually the two boxes
  // touch along an edge.)
  EXPECT_EQ(PointOnBoxSurfaceHelper(p_ACa, box_A), Vector3d(1, 0, 1));
  EXPECT_EQ(PointOnBoxSurfaceHelper(p_BCb, box_B), Vector3d(1, 1, 1));

  // It makes sense to have gradient mostly along +Wz direction because box_A
  // is above box_B (see the picture in issue 21192). There are other valid
  // directions.  If the code changes and selects a different direction, we
  // will have to change the expected result.
  const Vector3d nhat_BA_W =
      BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);
  EXPECT_TRUE(CompareMatrices(nhat_BA_W, Vector3d(0, -0.0850, 0.9963), 1e-4));
}

}  // namespace
}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
