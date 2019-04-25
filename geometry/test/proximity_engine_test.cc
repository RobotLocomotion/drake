#include "drake/geometry/proximity_engine.h"

#include <cmath>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
// Compare witness pair. Note that we can switch body A with body B in one pair,
// and the comparison result would be the same.
void CompareSignedDistancePair(const SignedDistancePair<double>& pair,
                               const SignedDistancePair<double>& pair_expected,
                               double tol) {
  EXPECT_NEAR(pair.distance, pair_expected.distance, tol);
  ASSERT_LT(pair.id_A, pair_expected.id_B);
  EXPECT_EQ(pair.id_B, pair_expected.id_B);
  EXPECT_TRUE(CompareMatrices(pair.p_ACa, pair_expected.p_ACa, tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(pair.p_BCb, pair_expected.p_BCb, tol,
                              MatrixCompareType::absolute));
}

class ProximityEngineTester {
 public:
  ProximityEngineTester() = delete;

  template <typename T>
  static bool IsDeepCopy(const ProximityEngine<T>& test_engine,
                         const ProximityEngine<T>& ref_engine) {
    return ref_engine.IsDeepCopy(test_engine);
  }

  template <typename T>
  static int peek_next_clique(const ProximityEngine<T>& engine) {
    return engine.peek_next_clique();
  }

  template <typename T>
  static Vector3<T> GetTranslation(ProximityIndex index, bool is_dynamic,
                                   const ProximityEngine<T>& engine) {
    return engine.GetX_WG(index, is_dynamic).translation();
  }

  template <typename T>
  static GeometryIndex GetGeometryIndex(ProximityIndex index, bool is_dynamic,
                                        const ProximityEngine<T>& engine) {
    return engine.GetGeometryIndex(index, is_dynamic);
  }
};

namespace {

using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::Vector2d;

using std::move;

// Tests for manipulating the population of the proximity engine.

// Test simple addition of dynamic geometry.
GTEST_TEST(ProximityEngineTests, AddDynamicGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  ProximityIndex index = engine.AddDynamicGeometry(sphere, GeometryIndex(0));
  EXPECT_EQ(index, 0);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests simple addition of anchored geometry.
GTEST_TEST(ProximityEngineTests, AddAnchoredGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex index = engine.AddAnchoredGeometry(sphere, pose,
                                                    GeometryIndex(0));
  EXPECT_EQ(index, 0);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 0);
}

// Tests addition of both dynamic and anchored geometry.
GTEST_TEST(ProximityEngineTests, AddMixedGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex a_index = engine.AddAnchoredGeometry(sphere, pose,
                                                      GeometryIndex(0));
  EXPECT_EQ(a_index, 0);
  ProximityIndex g_index = engine.AddDynamicGeometry(sphere, GeometryIndex(0));
  EXPECT_EQ(g_index, 0);
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Removes geometry (dynamic and anchored) from the engine. The test creates
// a _unique_ engine instance with all dynamic or all anchored geometries.
// It is not necessary to create a mixed engine because the two geometry
// types are segregated.
GTEST_TEST(ProximityEngineTests, RemoveGeometry) {
  for (bool is_dynamic : {true, false}) {
    ProximityEngine<double> engine;

    double x_pos[] = {0, 2, 4};

    std::vector<GeometryIndex> geometry_indices;
    std::vector<ProximityIndex> proximity_indices;
    std::vector<Isometry3<double>> poses;

    // Populate the world with three anchored spheres located on the x-axis at
    // x = 0, 2, & 4. With radius of 0.5, they should *not* be colliding.
    Sphere sphere{0.5};

    for (int i = 0; i < 3; ++i) {
      geometry_indices.push_back(GeometryIndex(i + 10));
      poses.push_back(Isometry3<double>::Identity());
      poses[i].translation() << x_pos[i], 0, 0;
      if (is_dynamic) {
        proximity_indices.push_back(
            engine.AddDynamicGeometry(sphere, geometry_indices[i]));
      } else {
        proximity_indices.push_back(
            engine.AddAnchoredGeometry(sphere, poses[i], geometry_indices[i]));
      }
      EXPECT_EQ(proximity_indices[i], i);
      EXPECT_NE(static_cast<int>(proximity_indices[i]),
                static_cast<int>(geometry_indices[i]));
    }
    EXPECT_EQ(engine.num_geometries(), 3);
    EXPECT_EQ(engine.num_anchored(), is_dynamic ? 0 : 3);
    EXPECT_EQ(engine.num_dynamic(), is_dynamic ? 3 : 0);

    if (is_dynamic) {
      // Poses for dynamic geometries need to be explicitly updated.
      std::vector<GeometryIndex> indices(poses.size());
      std::iota(indices.begin(), indices.end(), GeometryIndex(0));
      engine.UpdateWorldPoses(poses, indices);
    }

    // Case: Remove middle object, confirm that final gets moved.
    auto remove_index = ProximityIndex(1);
    optional<GeometryIndex> moved =
        engine.RemoveGeometry(remove_index, is_dynamic);
    // Confirm that a move is reported, that the moved object has its engine
    // index updated, and that there is "physical" evidence of the move (e.g.,
    // the correct, unique position).
    {
      EXPECT_EQ(engine.num_geometries(), 2);
      EXPECT_EQ(engine.num_anchored(), is_dynamic ? 0 : 2);
      EXPECT_EQ(engine.num_dynamic(), is_dynamic ? 2 : 0);
      EXPECT_TRUE(moved);
      EXPECT_EQ(*moved, geometry_indices[2]);
      EXPECT_TRUE(CompareMatrices(ProximityEngineTester::GetTranslation(
                                      remove_index, is_dynamic, engine),
                                  Vector3<double>{x_pos[2], 0, 0}, 0,
                                  MatrixCompareType::absolute));
      EXPECT_EQ(ProximityEngineTester::GetGeometryIndex(remove_index,
                                                        is_dynamic, engine),
                geometry_indices[2]);
    }

    // Case: Remove the last object, nothing should get moved.
    moved = engine.RemoveGeometry(remove_index, is_dynamic);
    // RemoveGeometry
    {
      EXPECT_EQ(engine.num_geometries(), 1);
      EXPECT_EQ(engine.num_anchored(), is_dynamic ? 0 : 1);
      EXPECT_EQ(engine.num_dynamic(), is_dynamic ? 1 : 0);
      EXPECT_FALSE(moved);
      EXPECT_TRUE(CompareMatrices(ProximityEngineTester::GetTranslation(
                                      ProximityIndex(0), is_dynamic, engine),
                                  Vector3<double>{x_pos[0], 0, 0}, 0,
                                  MatrixCompareType::absolute));
      EXPECT_EQ(ProximityEngineTester::GetGeometryIndex(ProximityIndex(0),
                                                        is_dynamic, engine),
                geometry_indices[0]);
    }
  }
}

// Tests for reading .obj files.------------------------------------------------

// Tests exception when we read an .obj file with two objects into Convex
GTEST_TEST(ProximityEngineTests, ExceptionTwoObjectsInObjFileForConvex) {
  ProximityEngine<double> engine;
  Convex convex{drake::FindResourceOrThrow(
      "drake/geometry/test/forbidden_two_cubes.obj"), 1.0};
  DRAKE_EXPECT_THROWS_MESSAGE(engine.AddDynamicGeometry(convex,
                                                        GeometryIndex(0)),
      std::runtime_error, ".*one and only one object.*");
}

// Tests for copy/move semantics.  ---------------------------------------------

// Tests the copy semantics of the ProximityEngine -- the copy is a complete,
// deep copy. Every type of shape specification must be included in this test.
GTEST_TEST(ProximityEngineTests, CopySemantics) {
  ProximityEngine<double> ref_engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();

  // NOTE: The GeometryIndex values are all lies; the values are arbitrary but
  // do not matter in the context of this test.
  ref_engine.AddAnchoredGeometry(sphere, pose, GeometryIndex(0));

  ref_engine.AddDynamicGeometry(sphere, GeometryIndex(1));

  Cylinder cylinder{0.1, 1.0};
  ref_engine.AddDynamicGeometry(cylinder, GeometryIndex(2));

  Box box{0.1, 0.2, 0.3};
  ref_engine.AddDynamicGeometry(box, GeometryIndex(3));

  HalfSpace halfspace{};
  ref_engine.AddDynamicGeometry(halfspace, GeometryIndex(4));

  Convex convex{drake::FindResourceOrThrow(
      "drake/geometry/test/quad_cube.obj"), 1.0};
  ref_engine.AddDynamicGeometry(convex, GeometryIndex(5));

  ProximityEngine<double> copy_construct(ref_engine);
  ProximityEngineTester::IsDeepCopy(copy_construct, ref_engine);

  ProximityEngine<double> copy_assign;
  copy_assign = ref_engine;
  ProximityEngineTester::IsDeepCopy(copy_assign, ref_engine);
}

// Tests the move semantics of the ProximityEngine -- the source is restored to
// default state.
GTEST_TEST(ProximityEngineTests, MoveSemantics) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex a_index = engine.AddAnchoredGeometry(sphere, pose,
                                                      GeometryIndex(0));
  EXPECT_EQ(a_index, 0);
  ProximityIndex g_index = engine.AddDynamicGeometry(sphere,
                                                     GeometryIndex(0));
  EXPECT_EQ(g_index, 0);

  ProximityEngine<double> move_construct(move(engine));
  EXPECT_EQ(move_construct.num_geometries(), 2);
  EXPECT_EQ(move_construct.num_anchored(), 1);
  EXPECT_EQ(move_construct.num_dynamic(), 1);
  EXPECT_EQ(engine.num_geometries(), 0);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 0);

  ProximityEngine<double> move_assign;
  move_assign = move(move_construct);
  EXPECT_EQ(move_assign.num_geometries(), 2);
  EXPECT_EQ(move_assign.num_anchored(), 1);
  EXPECT_EQ(move_assign.num_dynamic(), 1);
  EXPECT_EQ(move_construct.num_geometries(), 0);
  EXPECT_EQ(move_construct.num_anchored(), 0);
  EXPECT_EQ(move_construct.num_dynamic(), 0);
}

// Signed distance tests -- testing data flow; not testing the value of the
// query.

// A scene with no geometry reports no witness pairs.
GTEST_TEST(ProximityEngineTests, SignedDistanceClosestPointsOnEmptyScene) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> empty_map;

  const auto results =
      engine.ComputeSignedDistancePairwiseClosestPoints(empty_map);
  EXPECT_EQ(results.size(), 0);
}

// A scene with a single anchored geometry reports no distance.
GTEST_TEST(ProximityEngineTests, SignedDistanceClosestPointsSingleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> geometry_map;

  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex index = engine.AddAnchoredGeometry(sphere, pose,
                                                    GeometryIndex(0));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index, 0);
  const auto results = engine.ComputeSignedDistancePairwiseClosestPoints(
      geometry_map);
  EXPECT_EQ(results.size(), 0);
}

// Tests that anchored geometry don't report closest distance with each other.
GTEST_TEST(ProximityEngineTests, SignedDistanceClosestPointsMultipleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> geometry_map;

  const double radius = 0.5;
  Sphere sphere{radius};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex index1 = engine.AddAnchoredGeometry(sphere, pose,
                                                     GeometryIndex(0));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index1, 0);
  pose.translation() << 1.8 * radius, 0, 0;
  ProximityIndex index2 = engine.AddAnchoredGeometry(sphere, pose,
                                                     GeometryIndex(1));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index2, 1);
  const auto results = engine.ComputeSignedDistancePairwiseClosestPoints(
      geometry_map);
  EXPECT_EQ(results.size(), 0);
}

// ComputeSignedDistanceToPoint tests

using std::make_shared;
using std::shared_ptr;

// Test the broad-phase part of ComputeSignedDistanceToPoint.

// We put two small spheres with radius 0.1 centered at (1,1,1) and
// (-1,-1,-1). The query point Q will be at (3,3,3), so we can test that our
// code does call computeAABB() of the query point (by default, its AABB is
// [0,0]x[0,0]x[0,0]). We test several values of the distance threshold to
// include different numbers of spheres.
//
//                      Q query point
//
//
//        y
//        |    o first small sphere
//        |
//        +----- x
//
//    o second small sphere
//
GTEST_TEST(SignedDistanceToPointBroadphaseTest, MultipleThreshold) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> geometry_map;
  std::vector<Isometry3d> X_WGs;
  const double radius = 0.1;
  const Vector3d center1(1., 1., 1.);
  const Vector3d center2(-1, -1, -1.);
  int index = 0;
  for (const Vector3d p_WG : {center1, center2}) {
    const RigidTransformd X_WG(p_WG);
    X_WGs.push_back(X_WG.GetAsIsometry3());
    engine.AddAnchoredGeometry(Sphere(radius), X_WG.GetAsIsometry3(),
                               GeometryIndex(index++));
    geometry_map.push_back(GeometryId::get_new_id());
  }
  const Vector3d p_WQ(3., 3., 3.);
  // This small threshold allows no sphere.
  double threshold = 0.001;
  auto results = engine.ComputeSignedDistanceToPoint(p_WQ, geometry_map, X_WGs,
                                                     threshold);
  EXPECT_EQ(0, results.size());
  // This threshold touches the corner of the bounding box of the first sphere.
  // It is still too small to yield any result.
  threshold = (p_WQ - (center1 + Vector3d(radius, radius, radius))).norm();
  results =
      engine.ComputeSignedDistanceToPoint(p_WQ, geometry_map, X_WGs, threshold);
  EXPECT_EQ(0, results.size());
  // This threshold barely touches outside the first sphere, so it still gives
  // no result.
  threshold = (p_WQ - center1).norm() - radius - 1e-10;
  results =
      engine.ComputeSignedDistanceToPoint(p_WQ, geometry_map, X_WGs, threshold);
  EXPECT_EQ(0, results.size());
  // This threshold barely touches inside the first sphere, so it gives
  // one result.
  threshold = (p_WQ - center1).norm() - radius + 1e-10;
  results =
      engine.ComputeSignedDistanceToPoint(p_WQ, geometry_map, X_WGs, threshold);
  EXPECT_EQ(1, results.size());
  // This threshold touches the corner of the bounding box of the second
  // sphere, so it is still too small to allow the second sphere.
  threshold = (p_WQ - (center2 + Vector3d(radius, radius, radius))).norm();
  results =
      engine.ComputeSignedDistanceToPoint(p_WQ, geometry_map, X_WGs, threshold);
  EXPECT_EQ(1, results.size());
  // This threshold barely touches the outside the second sphere, so it still
  // gives one result.
  threshold = (p_WQ - center2).norm() - radius - 1e-10;
  results =
      engine.ComputeSignedDistanceToPoint(p_WQ, geometry_map, X_WGs, threshold);
  EXPECT_EQ(1, results.size());
  // This threshold barely touches inside the second sphere, so it starts to
  // give two results.
  threshold = (p_WQ - center2).norm() - radius + 1e-10;
  results =
      engine.ComputeSignedDistanceToPoint(p_WQ, geometry_map, X_WGs, threshold);
  EXPECT_EQ(2, results.size());
}

// Test the narrow-phase part of ComputeSignedDistanceToPoint.

// Parameter for the value-parameterized test fixture SignedDistanceToPointTest.
struct SignedDistanceToPointTestData {
  SignedDistanceToPointTestData(shared_ptr<Shape> geometry_in,
                                const RigidTransformd& X_WG_in,
                                const Vector3d& p_WQ_in,
                                const SignedDistanceToPoint<double>& expect_in)
      : geometry(geometry_in),
        X_WG(X_WG_in),
        p_WQ(p_WQ_in),
        expected_result(expect_in) {}

  // Google Test uses this operator to report the test data in the log file
  // when a test fails.
  friend std::ostream& operator<<(std::ostream& os,
                                  const SignedDistanceToPointTestData& obj) {
    return os << "{\n"
              << "  geometry: (not printed)\n"
              << "  X_WG: (not printed)\n"
              << "  p_WQ: " << obj.p_WQ.transpose() << "\n"
              << "  expected_result.p_GN: "
              << obj.expected_result.p_GN.transpose() << "\n"
              << "  expected_result.distance: " << obj.expected_result.distance
              << "\n"
              << "  expected_result.grad_W: "
              << obj.expected_result.grad_W.transpose() << "\n"
              << "}" << std::flush;
  }

  shared_ptr<Shape> geometry;
  const RigidTransformd X_WG;
  const Vector3d p_WQ;
  const SignedDistanceToPoint<double> expected_result;
};

std::vector<SignedDistanceToPointTestData> GenDistanceTestDataSphere(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  // We use these identities
  //           2^2   + 3^2    + 6^2   = 7^2         (1)
  //           0.2^2 + 0.3^2  + 0.6^2 = 0.7^2       (2)
  //           0.1^2 + 0.15^2 + 0.3^2 = 0.35^2      (3)
  // to set up the radius of the sphere geometry G and the position of the
  // query point Q in such a way that both the signed distance and the
  // position of the nearest point N can be expressed with fixed-point numbers.
  //
  // We will set a query point Q both outside G and inside G in such a way
  // that Q is collinear to the center Go of the sphere and the nearest point N.
  //
  //            Q
  //           /
  //     ooo  /
  //  o      N
  // o      Q  o
  // o    Go   o
  // o         o
  //  o       o
  //     ooo
  //
  // From (2), we will set the radius of G to 0.7, so later we will have
  // p_GN at (0.2, 0.3, 0.6). From (1), we will set p_GQ to (2,3,6), so it
  // will be outside G at the positive distance 7 - 0.7 = 6.3. From (3), we will
  // set p_GQ to (0.1, 0.15, 0.3), so it will be inside G at the negative
  // distance 0.35 - 0.7 = -0.35. Both positions of Q will have the same
  // nearest point N and the gradient vector.
  auto sphere = make_shared<Sphere>(0.7);
  std::vector<SignedDistanceToPointTestData> test_data{
      // p_GQ = (2,3,6) is outside G at the positive distance 6.3.
      {sphere, X_WG, X_WG * Vector3d{2.0, 3.0, 6.0},
       SignedDistanceToPoint<double>(
           GeometryId::get_new_id(), Vector3d(0.2, 0.3, 0.6), 6.3,
           X_WG.rotation() * Vector3d(2.0, 3.0, 6.0) / 7.0,
           true /* grad_W is well defined. */)},
      // p_GQ = (0.1,0.15,0.3) is inside G at the negative distance -0.35.
      {sphere, X_WG, X_WG * Vector3d{0.1, 0.15, 0.3},
       SignedDistanceToPoint<double>(
           GeometryId::get_new_id(), Vector3d(0.2, 0.3, 0.6), -0.35,
           X_WG.rotation() * Vector3d(2.0, 3.0, 6.0) / 7.0,
           true /* grad_W is well defined */)},
      // Reports an arbitrary gradient vector (as defined in the
      // QueryObject::ComputeSignedDistanceToPoint() documentation) at the
      // center of the sphere.
      {sphere, X_WG, X_WG * Vector3d{0.0, 0.0, 0.0},
       SignedDistanceToPoint<double>(GeometryId::get_new_id(),
                                     Vector3d(0.7, 0., 0.), -0.7,
                                     X_WG.rotation() * Vector3d(1.0, 0.0, 0.0),
                                     false /* grad_W is not well defined */)}};
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformSphere() {
  const RigidTransformd X_WG(RollPitchYawd(M_PI / 6., M_PI / 3., M_PI_2),
                             Vector3d{10., 11., 12.});
  return GenDistanceTestDataSphere(X_WG);
}

// We declare this function here, so we can call it from
// GenDistanceTestDataOutsideBox() below.
std::vector<SignedDistanceToPointTestData> GenDistanceTestDataBoxBoundary(
    const RigidTransformd& X_WG = RigidTransformd::Identity());

// Generates test data for a query point Q outside a box geometry G nearest
// to one of the 6 faces, the 12 edges, and the 8 vertices of the box.
// First we call GenDistanceTestDataBoxBoundary() to generate test data for
// query points on the box boundary. Then, we move the query point along the
// gradient vector by a unit distance.  In each case, the the nearest point to
// Q on ∂G stays the same, the signed distance becomes +1, and the gradient
// vector stays the same.
std::vector<SignedDistanceToPointTestData> GenDistanceTestDataOutsideBox(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const std::vector<SignedDistanceToPointTestData> test_data_box_boundary =
      GenDistanceTestDataBoxBoundary(X_WG);
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& data : test_data_box_boundary) {
    const shared_ptr<Shape> shape = data.geometry;
    // We expect the shape to be a box.
    DRAKE_DEMAND(dynamic_cast<Box*>(shape.get()) != nullptr);
    // The gradient grad_W has unit length by construction.
    const Vector3d p_WQ = data.p_WQ + data.expected_result.grad_W;
    const GeometryId& id = data.expected_result.id_G;
    const Vector3d& p_GN = data.expected_result.p_GN;
    const double distance = 1.;
    const Vector3d& grad_W = data.expected_result.grad_W;
    test_data.emplace_back(
        shape, X_WG, p_WQ,
        SignedDistanceToPoint<double>(id, p_GN, distance, grad_W,
                                      true /* grad_W is well defined */));
  }
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformOutsideBox() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3., M_PI / 6., M_PI_2),
                       Vector3d{10., 11., 12.});
  return GenDistanceTestDataOutsideBox(X_WG);
}

// Generates test data for a query point Q on the boundary ∂G of a box
// geometry G. Q can be at the 8 corners, at the midpoints of the 12 edges, or
// at the centers of the 6 faces of G. The set of all 26 positions can be
// expressed in G's frame as the following Cartesian product excluding the
// origin (3x3x3-1 = 26) :
//
//     p_GQ ∈ {-h(x),0,+h(x)} x {-h(y),0,+h(y)} x {-h(z),0,+h(z)} - {(0,0,0)},
//
// where h(x), h(y), and h(z) are the half width, half depth, and half height
// of G, respectively. We do not allow p_GQ=(0,0,0) because it is in the
// interior of G. The number of zeroes in p_GQ corresponds to the location at
// a corner (no zero, 3 non-zeroes), at the midpoint of an edge (1 zero,
// 2 non-zeroes), or at the center of a face (2 zeroes, 1 non-zero).
//     The positions above is parameterized by the sign vector s expressed in
// G's frame as:
//
//     s_G = (sx,sy,sz) ∈ {-1,0,+1} x {-1,0,+1} x {-1,0,+1} - {(0,0,0)},
//     p_GQ(s) = s_G ∘ h_G,
//
// where h_G = (h(x), h(y), h(z)), which is the vector from the origin Go to a
// vertex of G expressed in G's frame. The operator ∘ is the entrywise product
// (also known as Hadamard product): (a,b,c)∘(u,v,w) = (a*u, b*v, c*w).
//     In each case, Q is also its own nearest point on ∂G, the signed distance
// is always zero, and the gradient vector equals the normalized unit vector of
// the vector s.
std::vector<SignedDistanceToPointTestData> GenDistanceTestDataBoxBoundary(
    const RigidTransformd& X_WG) {
  auto box = make_shared<Box>(20., 30., 10.);
  const Vector3d h_G = box->size() / 2.;
  const double distance = 0.;
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const double sx : {-1., 0., 1.}) {
    for (const double sy : {-1., 0., 1.}) {
      for (const double sz : {-1., 0., 1.}) {
        // Skip the origin.
        if (sx == 0. && sy == 0. && sz == 0.) continue;
        const Vector3d s_G(sx, sy, sz);
        const Vector3d p_GQ = s_G.cwiseProduct(h_G);
        const Vector3d p_WQ = X_WG * p_GQ;
        // We create new id for each test case to help distinguish them.
        const GeometryId id = GeometryId::get_new_id();
        // Q is its own nearest point on ∂G.
        const Vector3d& p_GN = p_GQ;
        // Rotation matrix for transforming vector expression from G to world.
        const RotationMatrixd& R_WG = X_WG.rotation();
        const Vector3d grad_G = s_G.normalized();
        const Vector3d grad_W = R_WG * grad_G;
        // grad_W is not well defined if the query point Q is on the edge or
        // vertex of the box.
        const int num_dim_on_boundary = static_cast<int>(sx != 0) +
                                        static_cast<int>(sy != 0) +
                                        static_cast<int>(sz != 0);
        const bool is_grad_W_well_defined = num_dim_on_boundary <= 1;
        test_data.emplace_back(
            box, X_WG, p_WQ,
            SignedDistanceToPoint<double>(id, p_GN, distance, grad_W,
                                          is_grad_W_well_defined));
      }
    }
  }
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformBoxBoundary() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3., M_PI / 6., M_PI_2),
                       Vector3d{10., 11., 12.});
  return GenDistanceTestDataBoxBoundary(X_WG);
}

// Generates test data for a query point Q inside a box geometry G with a
// unique nearest point on the boundary ∂G. Unlike Q on ∂G or outside G that
// has 26 cases each, we have only 6 cases of Q inside G. In each case, Q is
// unambiguously closest to a single face.
//     The position of Q is parameterized by a chosen distance d and the
// outward unit normal vector s of the six faces of G. We calculate the
// center C of the face and offset C by the distance d inwards into G:
//
//         d    = min {h(x), h(y), h(z)} / 2,
//         s_G  ∈ {-x, +x, -y, +y, -z, +z}
//         p_GC = s_G ∘ h_G
//         p_GQ = p_GC - d * s_G,
//
// where h_G = (h(x), h(y), h(z)) is the vector of the half width, half depth,
// and half height of G. It is the vector from the origin Go to a corner
// of G expressed in G's frame. The opertor ∘ is the entrywise product:
// (a,b,c)∘(u,v,w) = (a*u, b*v, c*w).
//     The chosen d is small enough that Q at the inward normal offset from a
// face center is still unambiguously closest to that face.
//     In each case, the nearest point N on ∂G is C, the negative signed
// distance is -d, and the gradient vector is the face normal vector s.
std::vector<SignedDistanceToPointTestData> GenDistTestDataInsideBoxUnique(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  // Create a box [-10,10]x[-15,15]x[-5,5],
  auto box = make_shared<Box>(20., 30., 10.);
  const Vector3d h_G = box->size() / 2.;
  const double d = h_G.minCoeff() / 2.;
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const Vector3d unit_vector :
       {Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()}) {
    for (const double sign : {-1., 1.}) {
      // Unit face normal vector.
      const Vector3d s_G = sign * unit_vector;
      // Center of a face.
      const Vector3d p_GC = s_G.cwiseProduct(h_G);
      const Vector3d p_GQ = p_GC - d * s_G;
      const Vector3d p_WQ = X_WG * p_GQ;
      // We create new id for each test case to help distinguish them.
      const GeometryId id = GeometryId::get_new_id();
      // The nearest point is at the face center.
      const Vector3d& p_GN = p_GC;
      // Rotation matrix for transforming vector expression from G to world.
      const RotationMatrixd R_WG = X_WG.rotation();
      const Vector3d& grad_G = s_G;
      const Vector3d grad_W = R_WG * grad_G;
      test_data.emplace_back(
          box, X_WG, p_WQ,
          SignedDistanceToPoint<double>(id, p_GN, -d, grad_W,
                                        true /* grad_W is well defined */));
    }
  }
  return test_data;
}

// We test a rigid transform with the signed distance to query point Q inside
// a box B only when Q has a unique nearest point on the boundary ∂B.
std::vector<SignedDistanceToPointTestData>
GenDistTestTransformInsideBoxUnique() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3., M_PI / 6., M_PI_2),
                       Vector3d{10., 11., 12.});
  return GenDistTestDataInsideBoxUnique(X_WG);
}

// A query point Q inside a box G with multiple nearest points on the
// boundary ∂B needs a tie-breaking rule. However, a rigid transform can
// contaminate the tie breaking due to rounding errors.  We test the tie
// breaking with the identity transform only.
std::vector<SignedDistanceToPointTestData> GenDistTestDataInsideBoxNonUnique() {
  // We set up a 20x10x10 box [-10,10]x[-5,5]x[-5,5].  Having the same depth
  // and height allows Q to have five nearest faces in the last case below.
  auto box = make_shared<Box>(20., 10., 10.);
  const RigidTransformd& X_WG = RigidTransformd::Identity();
  std::vector<SignedDistanceToPointTestData> test_data{
      // Q is nearest to the face [-10,10]x[-5,5]x{5}.
      {box, X_WG, X_WG * Vector3d{6., 1., 2.},
       SignedDistanceToPoint<double>(GeometryId::get_new_id(),
                                     Vector3d(6., 1., 5.), -3.,
                                     X_WG.rotation() * Vector3d(0., 0., 1.),
                                     true /* grad_W is well defined */)},
      // Q is nearest to two faces {10}x[-5,5]x[-5,5] and [-10,10]x[-5,5]x{5}.
      {box, X_WG, X_WG * Vector3d{6., 0., 1.},
       SignedDistanceToPoint<double>(GeometryId::get_new_id(),
                                     Vector3d(10., 0., 1.), -4.,
                                     X_WG.rotation() * Vector3d(1., 0., 0.),
                                     true /* grad_W is well defined */)},
      // Q is nearest to three faces {10}x[-5,5]x[-5,5], [-10,10]x{5}x[-5,5],
      // and [-10,10]x[-5,5]x{5}.
      {box, X_WG, X_WG * Vector3d{6., 1., 1.},
       SignedDistanceToPoint<double>(GeometryId::get_new_id(),
                                     Vector3d(10., 1., 1.), -4.,
                                     X_WG.rotation() * Vector3d(1., 0., 0.),
                                     true /* grad_W is well defined */)},
      // Q at the center of the box is nearest to four faces
      // [-10,10]x{5}x[-5,5], [-10,10]x{-5}x[-5,5], [-10,10]x[5,-5]x{5},
      // and [-10,10]x[5,-5]x{-5}.
      {box, X_WG, X_WG * Vector3d{0., 0., 0.},
       SignedDistanceToPoint<double>(GeometryId::get_new_id(),
                                     Vector3d(0., 5., 0.), -5.,
                                     X_WG.rotation() * Vector3d(0., 1., 0.),
                                     true /* grad_W is well defined */)},
      // Q is nearest to five faces {10}x[-5,5]x[-5,5], [-10,10]x{5}x[-5,5],
      // [-10,10]x{-5}x[-5,5], [-10,10]x[5,-5]x{5}, and [-10,10]x[5,-5]x{-5}.
      {box, X_WG, X_WG * Vector3d{5., 0., 0.},
       SignedDistanceToPoint<double>(GeometryId::get_new_id(),
                                     Vector3d(10., 0., 0.), -5.,
                                     X_WG.rotation() * Vector3d(1., 0., 0.),
                                     true /* grad_W is well defined */)}};
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistanceTestDataTranslateBox() {
  // We set up a 20x10x30 box G centered at the origin [-10,10]x[-5,5]x[-15,15].
  auto box = make_shared<Box>(20., 10., 30.);
  std::vector<SignedDistanceToPointTestData> test_data{
      // We translate G by (10,20,30) to [0,20]x[15,25]x[15,45] in World frame.
      // The position of the query point p_WQ(23,20,30) is closest to G at
      // (20,20,30) in World frame, which is p_GN=(10,0,0) in G's frame.
      // The gradient vector is expressed in both frames as (1,0,0).
      {box, Translation3d(10., 20., 30.), Vector3d{23., 20., 30.},
       SignedDistanceToPoint<double>(
           GeometryId::get_new_id(), Vector3d(10., 0., 0.), 3.,
           Vector3d(1., 0., 0.), true /* grad_W is well defined.*/)}};
  return test_data;
}

// Generate test data for a query point Q from a cylinder G.

// We separate the test data for Q on the boundary ∂G into two parts: Q on
// the top/bottom circles (GenDistTestDataCylinderBoundaryCircle) and Q on
// the cap/barrel surfaces (GenDistTestDataCylinderBoundarySurface).
// Here, a circle is a 1-dimensional closed curve, and a cap is a
// 2-dimensional flat surface bounded by a circle.
//     We separate the two cases because we will generate Q outside/inside G
// by moving Q on ∂G outwards/inwards along its gradient vector. In most
// cases, we can move Q a small distance, and its nearest point N on ∂G stays
// the same, namely the original Q on ∂G. However, moving Q on the top/bottom
// circles inwards would change its nearest point N, and we don't want to do
// that.
//     Later we will combine them into GenDistTestDataCylinderBoundary which
// will be used by GenDistTestDataOutsideCylinder.
//     In summary, the call graph looks like this:
//
// GenDistTestDataInsideCylinder
// |
// |  GenDistTestDataOutsideCylinder
// |  |
// |  +--> GenDistTestDataCylinderBoundary
// |       |
// |       +--> GenDistTestDataCylinderBoundaryCircle
// |       |
// +-----> +--> GenDistTestDataCylinderBoundarySurface

// Generates test data for a query point Q on the top/bottom circles of a
// cylinder geometry G.
std::vector<SignedDistanceToPointTestData>
GenDistTestDataCylinderBoundaryCircle(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const RotationMatrixd& R_WG = X_WG.rotation();
  auto cylinder = make_shared<Cylinder>(3.0, 5.0);
  const double radius = cylinder->get_radius();
  const double half_length = cylinder->get_length() / 2.0;
  // We want the test to cover all the combinations of positive, negative,
  // and zero values of both x and y coordinates on the two boundary circles
  // of the cylinder. Furthermore, each (x,y) has |x| ≠ |y| to avoid
  // symmetry that might hide problems. We achieve this by having 12 points
  // equally spread around a unit circle and map them to the two boundary
  // circles later.
  const int kNumVectors = 12;
  // unit vectors in x-y plane of G's frame.
  std::vector<Vector3d> all_xy_vectors;
  const Vector3d kXVector(1., 0., 0.);
  for (int c = 0; c < kNumVectors; ++c) {
    all_xy_vectors.push_back(
        RotationMatrixd(RollPitchYawd(0., 0., 2.0 * M_PI * c / kNumVectors))
        * kXVector);
  }
  const double distance = 0.0;  // Q on ∂G has distance zero.
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& z_vector : {Vector3d(0., 0., 1.), Vector3d(0., 0., -1.)}) {
    for (const auto& xy_vector : all_xy_vectors) {
      const GeometryId id = GeometryId::get_new_id();
      // xy_vector and z_vector are unit vectors in x-y plane and z-axis of G.
      const Vector3d p_GQ = radius * xy_vector + half_length * z_vector;
      const Vector3d p_WQ = X_WG * p_GQ;
      // Q is its own nearest point on ∂G.
      const Vector3d p_GN = p_GQ;
      // We set the gradient vector according to the convention described in
      // QueryObject::ComputeSignedDistanceToPoint().  Mathematically it is
      // undefined.
      const Vector3d grad_G = (xy_vector + z_vector).normalized();
      const Vector3d grad_W = R_WG * grad_G;
      test_data.emplace_back(cylinder, X_WG, p_WQ,
                             SignedDistanceToPoint<double>(
                                 id, p_GN, distance, grad_W,
                                 false /* grad_W is not well defined */));
    }
  }
  return test_data;
}

// Generates test data for a query point Q on the boundary surface ∂G of a
// cylinder geometry G. Q can be on the barrel or on the top or bottom caps.
std::vector<SignedDistanceToPointTestData>
GenDistTestDataCylinderBoundarySurface(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const RotationMatrixd& R_WG = X_WG.rotation();
  auto cylinder = make_shared<Cylinder>(3.0, 5.0);
  const double radius = cylinder->get_radius();
  const double half_length = cylinder->get_length() / 2.0;
  // We want the test to cover all the combinations of positive, negative,
  // and zero values of both x and y coordinates on some circles on the
  // barrel or on the caps. Furthermore, each (x,y) has |x| ≠ |y| to avoid
  // symmetry that might hide problems. We achieve this goal by having
  // 12 points equally spread around a unit circle and map them to the barrel
  // or the caps later.
  const int kNumVectors = 12;
  // unit vectors in x-y plane of G's frame.
  std::vector<Vector3d> all_xy_vectors;
  const Vector3d kXVector(1., 0., 0.);
  for (int c = 0; c < kNumVectors; ++c) {
    all_xy_vectors.push_back(
        RotationMatrixd(RollPitchYawd(0., 0., 2.0 * M_PI * c / kNumVectors))
        * kXVector);
  }
  struct LocalTestData {
    LocalTestData(const Vector3d& p_GQ_in, const Vector3d& grad_G_in)
        : p_GQ(p_GQ_in), grad_G(grad_G_in) {}
    Vector3d p_GQ;
    Vector3d grad_G;
  };
  // Generate LocalTestData that will convert to
  // SignedDistanceToPointTestData later.
  std::vector<LocalTestData> test_data_barrel;
  std::vector<LocalTestData> test_data_caps;
  std::vector<LocalTestData> test_data_cap_centers;
  for (const auto& z_vector : {Vector3d(0., 0., 1.), Vector3d(0., 0., -1.)}) {
    // Q at the centers of the top and bottom caps.
    {
      // z_vector is a unit vector.
      const Vector3d p_GQ = half_length * z_vector;
      // The gradient vector on the circular disks is along the z-axis of G.
      const Vector3d grad_G = z_vector;
      test_data_cap_centers.emplace_back(p_GQ, grad_G);
    }
    for (const auto& xy_vector : all_xy_vectors) {
      // Q on the barrel.
      {
        // Control how far vertically Q is from x-y plane of G.
        const double kZFactor = 0.5;
        // xy_vector and z_vector are unit vectors in x-y plane and z-axis of G.
        const Vector3d p_GQ =
            radius * xy_vector + half_length * kZFactor * z_vector;
        // The gradient vector on the barrel is parallel to the x-y plane of G.
        const Vector3d grad_G = xy_vector;
        test_data_barrel.emplace_back(p_GQ, grad_G);
      }
      // Q on the top and bottom caps.
      {
        // Control how far radially Q is from z-axis of G.
        const double kRFactor = 0.6;
        // xy_vector and z_vector are unit vectors in x-y plane and z-axis of G.
        const Vector3d p_GQ =
            radius * kRFactor * xy_vector + half_length * z_vector;
        // The gradient vector on the caps is along the z-axis of G.
        const Vector3d grad_G = z_vector;
        test_data_caps.emplace_back(p_GQ, grad_G);
      }
    }
  }
  std::vector<SignedDistanceToPointTestData> test_data;
  // Convert LocalTestData to SignedDistanceToPointTestData and add to
  // test_data.
  auto convert = [&cylinder, &X_WG, &R_WG,
                  &test_data](const LocalTestData& local) {
    // Generate new id for each test record.
    const GeometryId id = GeometryId::get_new_id();
    const Vector3d p_WQ = X_WG * local.p_GQ;
    // Q on ∂G has distance zero.
    const double distance = 0.0;
    // Q is its own nearest point on ∂G.
    const Vector3d p_GN = local.p_GQ;
    const Vector3d grad_W = R_WG * local.grad_G;
    test_data.emplace_back(
        cylinder, X_WG, p_WQ,
        SignedDistanceToPoint<double>(id, p_GN, distance, grad_W,
                                      true /* grad_W is well defined */));
  };
  std::for_each(test_data_barrel.begin(), test_data_barrel.end(), convert);
  std::for_each(test_data_caps.begin(), test_data_caps.end(), convert);
  std::for_each(test_data_cap_centers.begin(), test_data_cap_centers.end(),
                convert);
  return test_data;
}

// Generates test data for a query point Q on the boundary of a cylinder
// geometry G. Combine GenDistTestDataCylinderBoundaryCircle with
// GenDistTestDataCylinderBoundarySurface.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCylinderBoundary(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  auto test_data = GenDistTestDataCylinderBoundaryCircle(X_WG);
  const auto test_data_boundary_surface =
      GenDistTestDataCylinderBoundarySurface(X_WG);
  for (const auto& data : test_data_boundary_surface) {
    test_data.emplace_back(data.geometry, data.X_WG, data.p_WQ,
                           data.expected_result);
  }
  return test_data;
}

// Generates test data for a query point Q outside a cylinder geometry G.
// First we call GenDistTestDataCylinderBoundary() to generate test data for
// query points on the boundary.  Then, we move the query point along the
// gradient vector by a unit distance outward. The nearest point to Q on ∂G
// stays the same, the signed distance becomes +1, and the gradient vector
// stays the same.
std::vector<SignedDistanceToPointTestData> GenDistTestDataOutsideCylinder(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const auto test_data_cylinder_boundary =
      GenDistTestDataCylinderBoundary(X_WG);
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& data : test_data_cylinder_boundary) {
    const shared_ptr<Shape>& shape = data.geometry;
    // We expect the shape to be a cylinder.
    DRAKE_DEMAND(dynamic_cast<Cylinder*>(shape.get()) != nullptr);
    // The gradient grad_W has unit length by construction.
    const Vector3d p_WQ = data.p_WQ + data.expected_result.grad_W;
    const GeometryId& id = data.expected_result.id_G;
    const Vector3d& p_GN = data.expected_result.p_GN;
    const double distance = 1.;
    const Vector3d& grad_W = data.expected_result.grad_W;
    test_data.emplace_back(
        shape, X_WG, p_WQ,
        SignedDistanceToPoint<double>(id, p_GN, distance, grad_W,
                                      true /* grad_W is well defined */));
  }
  return test_data;
}

// Generates test data for a query point Q inside a cylinder geometry G with
// a unique nearest point on the boundary ∂G. Unlike Q on ∂G or outside G, Q
// inside G cannot have its nearest point N on the top and bottom boundary
// circles; however, it can have N on the top and bottom cap surfaces.
// First we call GenDistTestDataCylinderBoundarySurface() to generate data on
// the boundary surface.  Then, we move the query point along the gradient
// vector a small negative distance into the interior.  The nearest point N
// stays the same, the signed distance becomes the negative distance, and the
// gradient vector stays the same.
std::vector<SignedDistanceToPointTestData> GenDistTestDataInsideCylinder(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const auto test_data_cylinder_boundary_surface =
      GenDistTestDataCylinderBoundarySurface(X_WG);
  std::vector<SignedDistanceToPointTestData> test_data;
  const double kNegativeDistance = -0.1;
  for (const auto& data : test_data_cylinder_boundary_surface) {
    const shared_ptr<Shape>& shape = data.geometry;
    // We expect the shape to be a cylinder.
    DRAKE_DEMAND(dynamic_cast<Cylinder*>(shape.get()) != nullptr);
    // The gradient grad_W has unit length by construction.
    const Vector3d p_WQ =
        data.p_WQ + kNegativeDistance * data.expected_result.grad_W;
    const GeometryId& id = data.expected_result.id_G;
    const Vector3d& p_GN = data.expected_result.p_GN;
    const Vector3d& grad_W = data.expected_result.grad_W;
    test_data.emplace_back(
        shape, X_WG, p_WQ,
        SignedDistanceToPoint<double>(id, p_GN, kNegativeDistance, grad_W,
                                      true /* grad_W is well defined */));
  }
  return test_data;
}

// Generates test data for a query point Q at the center of a long cylinder
// geometry G. Q's nearest point on ∂G is not unique. Our code picks the one
// on the x's axis of G's frame.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCylinderCenter(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const RotationMatrixd& R_WG = X_WG.rotation();
  auto long_cylinder = make_shared<Cylinder>(1.0, 20.0);
  const double radius = long_cylinder->get_radius();
  const GeometryId id = GeometryId::get_new_id();
  // The query point Q is at the center of the cylinder.
  const Vector3d p_GQ(0., 0., 0.);
  const Vector3d p_WQ = X_WG * p_GQ;
  const double distance = -radius;
  // The nearest point N is on the x's axis of G's frame by convention.
  const Vector3d p_GN = radius * Vector3d(1., 0., 0.);
  const Vector3d grad_G = Vector3d(1., 0., 0.);
  const Vector3d grad_W = R_WG * grad_G;
  std::vector<SignedDistanceToPointTestData> test_data;
  test_data.emplace_back(
      long_cylinder, X_WG, p_WQ,
      SignedDistanceToPoint<double>(id, p_GN, distance, grad_W,
                                    true /* grad_W is well defined */));
  return test_data;
}

// Generates test data for a query point Q on the boundary, inside, and
// outside a cylinder with a rigid transform.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCylinderTransform() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3., M_PI / 6., M_PI_2),
                       Vector3d{10., 11., 12.});
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& record : GenDistTestDataCylinderBoundary(X_WG))
    test_data.emplace_back(record);
  for (const auto& record : GenDistTestDataOutsideCylinder(X_WG))
    test_data.emplace_back(record);
  for (const auto& record : GenDistTestDataInsideCylinder(X_WG))
    test_data.emplace_back(record);
  for (const auto& record : GenDistTestDataCylinderCenter(X_WG))
    test_data.emplace_back(record);
  return test_data;
}

// This test fixture takes data generated by GenDistanceTestData*(),
// GenDistTestData*(), and GenDistTestTransform*() above.
struct SignedDistanceToPointTest
    : public testing::TestWithParam<SignedDistanceToPointTestData> {
  ProximityEngine<double> engine;
  std::vector<GeometryId> geometry_map;
  std::vector<Isometry3d> X_WGs;

  // The tolerance value for determining equivalency between expected and
  // tested results. The underlying algorithms have an empirically-determined,
  // hard-coded tolerance of 1e-14 to account for loss of precision due to
  // rigid transformations and this tolerance reflects that.
  static constexpr double tolerance = 1e-14;

  SignedDistanceToPointTest() {
    auto data = GetParam();
    engine.AddAnchoredGeometry(*(data.geometry), data.X_WG.GetAsIsometry3(),
                               GeometryIndex(0));
    X_WGs.push_back(data.X_WG.GetAsIsometry3());
    geometry_map.push_back(data.expected_result.id_G);
  }
};

TEST_P(SignedDistanceToPointTest, SingleQueryPoint) {
  auto data = GetParam();

  auto results =
      engine.ComputeSignedDistanceToPoint(data.p_WQ, geometry_map, X_WGs);
  EXPECT_EQ(results.size(), 1);
  EXPECT_EQ(results[0].id_G, data.expected_result.id_G);
  EXPECT_TRUE(
      CompareMatrices(results[0].p_GN, data.expected_result.p_GN, tolerance))
      << "Incorrect nearest point.";
  EXPECT_NEAR(results[0].distance, data.expected_result.distance, tolerance)
      << "Incorrect signed distance.";
  EXPECT_TRUE(CompareMatrices(results[0].grad_W, data.expected_result.grad_W,
                              tolerance))
      << "Incorrect gradient vector.";
}

TEST_P(SignedDistanceToPointTest, SingleQueryPointWithThreshold) {
  auto data = GetParam();

  const double large_threshold = data.expected_result.distance + 0.01;
  auto results =
      engine.ComputeSignedDistanceToPoint(data.p_WQ, geometry_map, X_WGs,
                                          large_threshold);
  // The large threshold allows one object in the results.
  EXPECT_EQ(results.size(), 1);

  const double small_threshold = data.expected_result.distance - 0.01;
  results =
      engine.ComputeSignedDistanceToPoint(data.p_WQ, geometry_map, X_WGs,
                                          small_threshold);
  // The small threshold skips all objects.
  EXPECT_EQ(results.size(), 0);
}


// To debug a specific test, you can use Bazel flag --test_filter and
// --test_output.  For example, you can use the command:
// ```
//   bazel test //geometry:proximity_engine_test
//       --test_filter=Sphere/SignedDistanceToPointTest.SingleQueryPoint/0
//       --test_output=all
// ```
// to run the first case from the test data generated by
// GenDistanceTestDataSphere() with the function
// TEST_P(SignedDistanceToPointTest, SingleQueryPoint).
// Sphere
INSTANTIATE_TEST_CASE_P(Sphere, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistanceTestDataSphere()));
INSTANTIATE_TEST_CASE_P(TransformSphere, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistTestTransformSphere()));
// Box
INSTANTIATE_TEST_CASE_P(OutsideBox, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistanceTestDataOutsideBox()));
INSTANTIATE_TEST_CASE_P(BoxBoundary, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistanceTestDataBoxBoundary()));
INSTANTIATE_TEST_CASE_P(InsideBoxUnique, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistTestDataInsideBoxUnique()));
INSTANTIATE_TEST_CASE_P(InsideBoxNonUnique, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistTestDataInsideBoxNonUnique()));
INSTANTIATE_TEST_CASE_P(TranslateBox, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistanceTestDataTranslateBox()));
INSTANTIATE_TEST_CASE_P(TransformOutsideBox, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistTestTransformOutsideBox()));
INSTANTIATE_TEST_CASE_P(TransformBoxBoundary, SignedDistanceToPointTest,
                        testing::ValuesIn(GenDistTestTransformBoxBoundary()));
INSTANTIATE_TEST_CASE_P(
    TransformInsideBoxUnique, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestTransformInsideBoxUnique()));
// Cylinder
INSTANTIATE_TEST_CASE_P(
    CylinderBoundary, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestDataCylinderBoundary()));
INSTANTIATE_TEST_CASE_P(
    OutsideCylinder, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestDataOutsideCylinder()));
INSTANTIATE_TEST_CASE_P(
    InsideCylinder, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestDataInsideCylinder()));
INSTANTIATE_TEST_CASE_P(
    CenterCylinder, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestDataCylinderCenter()));
INSTANTIATE_TEST_CASE_P(
    CylinderTransform, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestDataCylinderTransform()));

// Penetration tests -- testing data flow; not testing the value of the query.

// A scene with no geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationOnEmptyScene) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> empty_map;

  auto results = engine.ComputePointPairPenetration(empty_map);
  EXPECT_EQ(results.size(), 0);
}

// A scene with a single anchored geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationSingleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> geometry_map;

  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex index = engine.AddAnchoredGeometry(sphere, pose,
                                                    GeometryIndex(0));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index, 0);
  auto results = engine.ComputePointPairPenetration(geometry_map);
  EXPECT_EQ(results.size(), 0);
}

// Tests that anchored geometry aren't collided against each other -- even if
// they actually *are* in penetration.
GTEST_TEST(ProximityEngineTests, PenetrationMultipleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> geometry_map;

  const double radius = 0.5;
  Sphere sphere{radius};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex index1 = engine.AddAnchoredGeometry(sphere, pose,
                                                     GeometryIndex(0));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index1, 0);
  pose.translation() << 1.8 * radius, 0, 0;
  ProximityIndex index2 = engine.AddAnchoredGeometry(sphere, pose,
                                                     GeometryIndex(1));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index2, 1);
  auto results = engine.ComputePointPairPenetration(geometry_map);
  EXPECT_EQ(results.size(), 0);
}

// These tests validate collisions/distance between spheres. This does *not*
// test against other geometry types because we assume FCL works. This merely
// confirms that the ProximityEngine functions provide the correct mapping.

// Common class for evaluating a simple penetration case between two spheres.
// This tests that collisions that are expected are reported between
//   1. dynamic-anchored
//   2. dynamic-dynamic
// It uses spheres to confirm that the collision results are as expected.
// These are the only sphere-sphere tests because the assumption is that if you
// can get *any* sphere-sphere collision test right, you can get all of them
// right.
// NOTE: FCL does not document the case where two spheres have coincident
// centers; therefore it is not tested here.
class SimplePenetrationTest : public ::testing::Test {
 protected:
  // Moves the dynamic sphere to either a penetrating or non-penetrating
  // position. The sphere is indicated by its engine `index` which belongs to
  // the given `source_id`. If `is_colliding` is true, the sphere is placed in
  // a colliding configuration.
  //
  // Non-colliding state
  //       y           x = free_x_
  //        │          │
  //       *│*         o o
  //    *   │   *   o       o
  //   *    │    * o         o
  // ──*────┼────*─o─────────o───────── x
  //   *    │    * o         o
  //    *   │   *   o       o
  //       *│*         o o
  //
  // Colliding state
  //       y       x = colliding_x_
  //        │      │
  //       *│*    o o
  //    *   │  o*      o
  //   *    │ o  *      o
  // ──*────┼─o──*──────o────────────── x
  //   *    │ o  *      o
  //    *   │  o*      o
  //       *│*    o o
  void MoveDynamicSphere(int index, bool is_colliding,
                         ProximityEngine<double>* engine = nullptr) {
    engine = engine == nullptr ? &engine_ : engine;
    std::vector<Isometry3<double>> poses(engine->num_dynamic(),
                                         Isometry3<double>::Identity());
    const double x_pos = is_colliding ? colliding_x_ : free_x_;
    poses[index] = Isometry3<double>(Translation3d{x_pos, 0, 0});
    std::vector<GeometryIndex> indices(poses.size());
    std::iota(indices.begin(), indices.end(), GeometryIndex(0));
    engine->UpdateWorldPoses(poses, indices);
  }

  // Compute penetration and confirm that a single penetration with the expected
  // properties was found. Provide the engine indices of the sphere located at
  // the origin and the sphere positioned to be in collision.
  template <typename T>
  void ExpectPenetration(GeometryId origin_sphere, GeometryId colliding_sphere,
                         ProximityEngine<T>* engine) {
    std::vector<PenetrationAsPointPair<double>> penetration_results =
        engine->ComputePointPairPenetration(geometry_map_);
    ASSERT_EQ(penetration_results.size(), 1);
    const PenetrationAsPointPair<double>& penetration = penetration_results[0];

    std::vector<SignedDistancePair<double>> distance_results =
        engine->ComputeSignedDistancePairwiseClosestPoints(geometry_map_);
    ASSERT_EQ(distance_results.size(), 1);
    const SignedDistancePair<double>& distance = distance_results[0];

    // There are no guarantees as to the ordering of which element is A and
    // which is B. This test enforces an order for validation.

    // First confirm membership
    EXPECT_TRUE((penetration.id_A == origin_sphere &&
                 penetration.id_B == colliding_sphere) ||
                (penetration.id_A == colliding_sphere &&
                 penetration.id_B == origin_sphere));

    EXPECT_TRUE(
        (distance.id_A == origin_sphere && distance.id_B == colliding_sphere) ||
        (distance.id_A == colliding_sphere && distance.id_B == origin_sphere));

    // Assume A => origin_sphere and b => colliding_sphere
    // NOTE: In this current version, penetration is only reported in double.
    PenetrationAsPointPair<double> expected;
    // This implicitly tests the *ordering* of the two reported ids. It must
    // always be in *this* order.
    bool origin_is_A = origin_sphere < colliding_sphere;
    expected.id_A = origin_is_A ? origin_sphere : colliding_sphere;
    expected.id_B = origin_is_A ? colliding_sphere : origin_sphere;
    expected.depth = 2 * radius_ - colliding_x_;
    // Contact point on the origin_sphere.
    Vector3<double> p_WCo{radius_, 0, 0};
    // Contact point on the colliding_sphere.
    Vector3<double> p_WCc{colliding_x_ - radius_, 0, 0};
    expected.p_WCa = origin_is_A ? p_WCo : p_WCc;
    expected.p_WCb = origin_is_A ? p_WCc : p_WCo;
    Vector3<double> norm_into_B = Vector3<double>::UnitX();
    expected.nhat_BA_W = origin_is_A ? -norm_into_B : norm_into_B;

    EXPECT_EQ(penetration.id_A, expected.id_A);
    EXPECT_EQ(penetration.id_B, expected.id_B);
    EXPECT_EQ(penetration.depth, expected.depth);
    EXPECT_TRUE(CompareMatrices(penetration.p_WCa, expected.p_WCa, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.p_WCb, expected.p_WCb, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.nhat_BA_W, expected.nhat_BA_W,
                                1e-13, MatrixCompareType::absolute));

    // Should return the penetration depth here.
    SignedDistancePair<double> expected_distance;
    expected_distance.distance = -expected.depth;
    expected_distance.id_A = origin_sphere;
    expected_distance.id_B = colliding_sphere;
    expected_distance.p_ACa = Eigen::Vector3d(radius_, 0, 0);
    expected_distance.p_BCb = Eigen::Vector3d(-radius_, 0, 0);
    if (expected_distance.id_B < expected_distance.id_A) {
      std::swap(expected_distance.id_A, expected_distance.id_B);
      std::swap(expected_distance.p_ACa, expected_distance.p_BCb);
    }
    EXPECT_LT(distance.id_A, distance.id_B);
    CompareSignedDistancePair(distance, expected_distance,
        std::numeric_limits<double>::epsilon());
  }

  // The two spheres collides, but are ignored due to the setting in the
  // collision filter.
  template <typename T>
  void ExpectIgnoredPenetration(GeometryId origin_sphere,
                                GeometryId colliding_sphere,
                                ProximityEngine<T>* engine) {
    std::vector<PenetrationAsPointPair<double>> penetration_results =
        engine->ComputePointPairPenetration(geometry_map_);
    EXPECT_EQ(penetration_results.size(), 0);

    std::vector<SignedDistancePair<double>> distance_results =
        engine->ComputeSignedDistancePairwiseClosestPoints(geometry_map_);
    ASSERT_EQ(distance_results.size(), 0);
  }

  // Compute penetration and confirm that none were found.
  template <typename T>
  void ExpectNoPenetration(GeometryId origin_sphere,
                           GeometryId colliding_sphere,
                           ProximityEngine<T>* engine) {
    std::vector<PenetrationAsPointPair<double>> penetration_results =
        engine->ComputePointPairPenetration(geometry_map_);
    EXPECT_EQ(penetration_results.size(), 0);

    std::vector<SignedDistancePair<double>> distance_results =
        engine->ComputeSignedDistancePairwiseClosestPoints(geometry_map_);
    ASSERT_EQ(distance_results.size(), 1);
    SignedDistancePair<double> distance = distance_results[0];

    // There are no guarantees as to the ordering of which element is A and
    // which is B. This test enforces an order for validation.
    EXPECT_TRUE(
        (distance.id_A == origin_sphere && distance.id_B == colliding_sphere) ||
        (distance.id_A == colliding_sphere && distance.id_B == origin_sphere));

    bool origin_is_A = origin_sphere < colliding_sphere;
    SignedDistancePair<double> expected_distance;
    expected_distance.id_A = origin_is_A ? origin_sphere : colliding_sphere;
    expected_distance.id_B = origin_is_A ? colliding_sphere : origin_sphere;
    expected_distance.distance = free_x_ - 2 * radius_;
    // Contact point on the origin_sphere.
    Vector3<double> p_OCo{radius_, 0, 0};
    // Contact point on the colliding_sphere.
    Vector3<double> p_CCc{-radius_, 0, 0};
    expected_distance.p_ACa = origin_is_A ? p_OCo : p_CCc;
    expected_distance.p_BCb = origin_is_A ? p_CCc : p_OCo;

    CompareSignedDistancePair(distance, expected_distance,
        std::numeric_limits<double>::epsilon());
  }

  ProximityEngine<double> engine_;
  std::vector<GeometryId> geometry_map_;
  const double radius_{0.5};
  const Sphere sphere_{radius_};
  const double free_x_{2.5 * radius_};
  const double colliding_x_{1.5 * radius_};
};

// Tests collision between dynamic and anchored sphere. One case colliding, one
// case *not* colliding.
TEST_F(SimplePenetrationTest, PenetrationDynamicAndAnchored) {
  // Set up anchored geometry
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex anchored_index =
      engine_.AddAnchoredGeometry(sphere_, pose, GeometryIndex(0));
  EXPECT_EQ(anchored_index, 0);
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);

  // Set up dynamic geometry
  ProximityIndex dynamic_index =
      engine_.AddDynamicGeometry(sphere_, GeometryIndex(1));
  EXPECT_EQ(dynamic_index, 0);
  GeometryId dynamic_id = GeometryId::get_new_id();
  geometry_map_.push_back(dynamic_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  // Non-colliding case
  MoveDynamicSphere(dynamic_index, false /* not colliding */);
  ExpectNoPenetration(origin_id, dynamic_id, &engine_);

  // Colliding case
  MoveDynamicSphere(dynamic_index, true /* colliding */);
  ExpectPenetration(origin_id, dynamic_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(origin_id, dynamic_id, &copy_engine);

  // Test AutoDiffXd converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToAutoDiffXd();
  ExpectPenetration(origin_id, dynamic_id, ad_engine.get());
}

// Performs the same collision test between two dynamic spheres which belong to
// the same source
TEST_F(SimplePenetrationTest, PenetrationDynamicAndDynamicSingleSource) {
  ProximityIndex origin_index =
      engine_.AddDynamicGeometry(sphere_, GeometryIndex(0));
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);
  EXPECT_EQ(origin_index, 0);
  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  std::vector<GeometryIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), GeometryIndex(0));
  engine_.UpdateWorldPoses(poses, indices);

  ProximityIndex collide_index =
      engine_.AddDynamicGeometry(sphere_, GeometryIndex(1));
  GeometryId collide_id = GeometryId::get_new_id();
  geometry_map_.push_back(collide_id);
  EXPECT_EQ(collide_index, 1);
  EXPECT_EQ(engine_.num_geometries(), 2);

  // Non-colliding case
  MoveDynamicSphere(collide_index, false /* not colliding */);
  ExpectNoPenetration(origin_id, collide_id, &engine_);

  // Colliding case
  MoveDynamicSphere(collide_index, true /* colliding */);
  ExpectPenetration(origin_id, collide_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(origin_id, collide_id, &copy_engine);

  // Test AutoDiffXd converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToAutoDiffXd();
  ExpectPenetration(origin_id, collide_id, ad_engine.get());
}

// Invokes ExcludeCollisionsWithin in various scenarios which will and won't
// generate cliques.
TEST_F(SimplePenetrationTest, ExcludeCollisionsWithinCliqueGeneration) {
  using PET = ProximityEngineTester;
  GeometryIndex dynamic1(0);
  engine_.AddDynamicGeometry(sphere_, dynamic1);
  GeometryIndex dynamic2(1);
  engine_.AddDynamicGeometry(sphere_, dynamic2);

  Isometry3<double> pose{Isometry3<double>::Identity()};
  GeometryIndex anchored1(0);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored1);
  GeometryIndex anchored2(1);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored2);

  int expected_clique = PET::peek_next_clique(engine_);

  // Named aliases for otherwise inscrutable true/false magic values. The
  // parameter in the invoked method is called `is_dynamic`. So, we set the
  // the constant `is_dynamic` to true and its opposite, `is_anchored` to false.
  const bool is_anchored = false;
  const bool is_dynamic = true;

  // No dynamic geometry --> no cliques generated.
  engine_.ExcludeCollisionsWithin({}, {anchored1, anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);
  EXPECT_TRUE(engine_.CollisionFiltered(anchored1, is_anchored,
                                        anchored2, is_anchored));

  // Single dynamic and no anchored geometry --> no cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1}, {});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // Multiple dynamic and no anchored geometry --> cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1, dynamic2}, {});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
  EXPECT_TRUE(engine_.CollisionFiltered(dynamic1, is_dynamic,
                                        dynamic2, is_dynamic));

  // Single dynamic and (one or more) anchored geometry --> cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1}, {anchored1});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
  EXPECT_TRUE(engine_.CollisionFiltered(anchored1, is_anchored,
                                        dynamic1, is_dynamic));
  engine_.ExcludeCollisionsWithin({dynamic1}, {anchored1, anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
  EXPECT_TRUE(engine_.CollisionFiltered(anchored2, is_anchored,
                                        dynamic1, is_dynamic));

  // Multiple dynamic and (one or more) anchored geometry --> cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1, dynamic2}, {anchored1});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
  engine_.ExcludeCollisionsWithin({dynamic1, dynamic2}, {anchored1, anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
}

// Performs the same collision test where the geometries have been filtered.
TEST_F(SimplePenetrationTest, ExcludeCollisionsWithin) {
  GeometryIndex origin_index(0);
  engine_.AddDynamicGeometry(sphere_, origin_index);
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);

  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  std::vector<GeometryIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), GeometryIndex(0));
  engine_.UpdateWorldPoses(poses, indices);

  GeometryIndex collide_index(1);
  engine_.AddDynamicGeometry(sphere_, collide_index);
  GeometryId collide_id = GeometryId::get_new_id();
  geometry_map_.push_back(collide_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  EXPECT_FALSE(engine_.CollisionFiltered(origin_index, true,
                                         collide_index, true));
  engine_.ExcludeCollisionsWithin({origin_index, collide_index}, {});
  EXPECT_TRUE(engine_.CollisionFiltered(origin_index, true,
                                        collide_index, true));

  // Non-colliding case
  MoveDynamicSphere(collide_index, false /* not colliding */);
  ExpectIgnoredPenetration(origin_id, collide_id, &engine_);

  // Colliding case
  MoveDynamicSphere(collide_index, true /* colliding */);
  ExpectIgnoredPenetration(origin_id, collide_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectIgnoredPenetration(origin_id, collide_id, &copy_engine);

  // Test AutoDiffXd converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToAutoDiffXd();
  ExpectIgnoredPenetration(origin_id, collide_id, ad_engine.get());
}

// Invokes ExcludeCollisionsBetween in various scenarios which will and won't
// generate cliques.
TEST_F(SimplePenetrationTest, ExcludeCollisionsBetweenCliqueGeneration) {
  using PET = ProximityEngineTester;
  GeometryIndex dynamic1(0);
  engine_.AddDynamicGeometry(sphere_, dynamic1);
  GeometryIndex dynamic2(1);
  engine_.AddDynamicGeometry(sphere_, dynamic2);
  GeometryIndex dynamic3(2);
  engine_.AddDynamicGeometry(sphere_, dynamic3);

  Isometry3<double> pose{Isometry3<double>::Identity()};
  GeometryIndex anchored1(0);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored1);
  GeometryIndex anchored2(1);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored2);
  GeometryIndex anchored3(2);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored3);

  int expected_clique = PET::peek_next_clique(engine_);

  // No dynamic geometry --> no cliques generated.
  engine_.ExcludeCollisionsBetween({}, {anchored1}, {}, {anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // One empty group --> no cliques generated
  engine_.ExcludeCollisionsBetween({}, {}, {dynamic1}, {anchored1});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);
  engine_.ExcludeCollisionsBetween({dynamic1}, {anchored1}, {}, {});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // Two groups with the same single geometry --> no cliques generated.
  engine_.ExcludeCollisionsBetween({dynamic1}, {}, {dynamic1}, {});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // Groups with dynamic and anchored geometry -- cliques generated for (g, a)
  // pairs but *not* (a, a) pairs: (d1, d2), (d1, a2), (d2, a1).
  engine_.ExcludeCollisionsBetween({dynamic1}, {anchored1}, {dynamic2},
                                   {anchored2});
  expected_clique += 3;
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // Repeat previous filter declaration -- no cliques added.
  engine_.ExcludeCollisionsBetween({dynamic1}, {anchored1}, {dynamic2},
                                   {anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // Partial repeat -- add one anchored geometry to one set. One new clique for
  // (d2, a3).
  engine_.ExcludeCollisionsBetween({dynamic1}, {anchored1, anchored3},
                                   {dynamic2}, {anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);

  // Partial repeat -- add one dynamic geometry to one set. Two new cliques for
  // (d3, d2) and (d3, a2).
  engine_.ExcludeCollisionsBetween({dynamic1, dynamic3}, {anchored1},
                                   {dynamic2}, {anchored2});
  expected_clique += 2;
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);
}

TEST_F(SimplePenetrationTest, ExcludeCollisionsBetween) {
  GeometryIndex origin_index(0);
  engine_.AddDynamicGeometry(sphere_, origin_index);
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);

  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  std::vector<GeometryIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), GeometryIndex(0));
  engine_.UpdateWorldPoses(poses, indices);

  GeometryIndex collide_index(1);
  engine_.AddDynamicGeometry(sphere_, collide_index);
  GeometryId collide_id = GeometryId::get_new_id();
  geometry_map_.push_back(collide_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  EXPECT_FALSE(engine_.CollisionFiltered(origin_index, true,
                                         collide_index, true));
  engine_.ExcludeCollisionsBetween({origin_index}, {}, {collide_index}, {});
  EXPECT_TRUE(engine_.CollisionFiltered(origin_index, true,
                                        collide_index, true));

  // Non-colliding case
  MoveDynamicSphere(collide_index, false /* not colliding */);
  ExpectIgnoredPenetration(origin_id, collide_id, &engine_);

  // Colliding case
  MoveDynamicSphere(collide_index, true /* colliding */);
  ExpectIgnoredPenetration(origin_id, collide_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectIgnoredPenetration(origin_id, collide_id, &copy_engine);

  // Test AutoDiffXd converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToAutoDiffXd();
  ExpectIgnoredPenetration(origin_id, collide_id, ad_engine.get());
}

// Test ComputeSignedDistancePairwiseClosestPoints with sphere-sphere,
// sphere-box, and sphere-cylinder pairs.  The definition of this test suite
// consists of four sections.
// 1. Generate test data as a vector of SignedDistancePairTestData. Each
//    record consists of both input and expected result.  See the function
//    GenDistancePairTestSphereSphere(), for example.
// 2. Define a test fixture parameterized by the test data.  It initializes
//    each test according to the test data. See the class
//    SignedDistancePairTest below.
// 3. Define one or more test procedures for the same test fixture. We call
//    ComputeSignedDistancePairwiseClosestPoints() from here.
//    See TEST_P(SignedDistancePairTest, SinglePair), for example.
// 4. Initiate all the tests by specifying the test fixture and the test data.
//    See INSTANTIATE_TEST_CASE_P() below.
class SignedDistancePairTestData {
 public:
  SignedDistancePairTestData(shared_ptr<const Shape> a,
                             shared_ptr<const Shape> b,
                             const RigidTransformd& X_WA,
                             const RigidTransformd& X_WB,
                             const SignedDistancePair<double>& expect)
      : a_(a),
        b_(b),
        X_WA_(X_WA),
        X_WB_(X_WB),
        expected_result_(expect) {}

  // Generates new test data by swapping geometry A and geometry B. It will
  // help us test the symmetric interface. For example, we can generate the
  // test data for test(box_B, sphere_A) from the test data for
  // test(sphere_A, box_B).
  SignedDistancePairTestData GenSwapAB() const {
    auto& id_A = expected_result_.id_A;
    auto& id_B = expected_result_.id_B;
    auto& p_ACa = expected_result_.p_ACa;
    auto& p_BCb = expected_result_.p_BCb;
    auto& distance = expected_result_.distance;
    return SignedDistancePairTestData(
        b_, a_, X_WB_, X_WA_,
        SignedDistancePair<double>(id_B, id_A, p_BCb, p_ACa, distance));
  }

  // Google Test uses this operator to report the test data in the log file
  // when a test fails.
  friend std::ostream& operator<<(std::ostream& os,
                                  const SignedDistancePairTestData& obj) {
    return os << "{\n"
              << " geometry A: (not printed)\n"
              << " geometry B: (not printed)\n"
              << " X_WA: (not printed)\n"
              << " X_WB: (not printed)\n"
              << " expected_result.id_A: "
              << obj.expected_result_.id_A << "\n"
              << " expected_result.id_B: "
              << obj.expected_result_.id_B << "\n"
              << " expected_result.distance: "
              << obj.expected_result_.distance << "\n"
              << " expected_result.p_ACa: "
              << obj.expected_result_.p_ACa.transpose() << "\n"
              << " expected_result.p_BCb: "
              << obj.expected_result_.p_BCb.transpose() << "\n"
              << "}" << std::flush;
  }

  shared_ptr<const Shape> a_;
  shared_ptr<const Shape> b_;
  const RigidTransformd X_WA_;
  const RigidTransformd X_WB_;
  const SignedDistancePair<double> expected_result_;
};

// Two spheres with varying degrees of overlapping.  The first sphere A is
// smaller than the second sphere B. We model the configurations in the frame
// of the first sphere A.  First we place B's center far enough to the right
// on the positive x-axis of A's frame that A and B do not overlap. Then, we
// move B's center towards A's center along the x-axis of A's frame until B
// covers A. The expressions of the witness points Ca ∈ ∂A and Cb ∈ ∂B in
// A's frame and B's frame respectively do not change during this motion.
//
// @param X_WA specifies the pose of A in world.
// @param R_WB specifies the orientation of B in world.
std::vector<SignedDistancePairTestData> GenDistancePairTestSphereSphere(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(1.0);
  auto sphere_B = make_shared<const Sphere>(2.0);
  double radius_A = sphere_A->get_radius();
  double radius_B = sphere_B->get_radius();
  // Set up R_AB and R_BA from X_WA and R_WB.
  const RotationMatrixd& R_WA = X_WA.rotation();
  const RotationMatrixd R_AW = R_WA.transpose();
  const RotationMatrixd R_AB = R_AW * R_WB;
  const RotationMatrixd R_BA = R_AB.transpose();
  struct Configuration {
    // Center of B in A's frame
    Vector3d p_ABo;
    double pair_distance;
  };
  const std::vector<Configuration> configurations {
      // Non-overlapping
      {Vector3d(4., 0., 0.), 1.},
      // B kisses A.
      {Vector3d(3., 0., 0.), 0.},
      // B overlaps A.
      {Vector3d(2.5, 0., 0.), -0.5},
      // B covers A.
      {Vector3d(1, 0., 0.), -2.}};
  const Vector3d p_ACa_A(radius_A, 0., 0.);
  // Position from Bo to Cb expressed in A's frame.
  const Vector3d p_BCb_A(-radius_B, 0., 0.);
  const Vector3d p_BCb_B = R_BA * p_BCb_A;
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& config : configurations) {
    const RigidTransformd X_AB(R_AB, config.p_ABo);
    const RigidTransformd X_WB = X_WA * X_AB;
    test_data.emplace_back(
        sphere_A, sphere_B, X_WA, X_WB,
        SignedDistancePair<double>(GeometryId::get_new_id(),
                                   GeometryId::get_new_id(), p_ACa_A, p_BCb_B,
                                   config.pair_distance));
  }
  return test_data;
}

std::vector<SignedDistancePairTestData> GenDistPairTestSphereSphereTransform() {
  return GenDistancePairTestSphereSphere(
      RigidTransformd(RollPitchYawd(5. * M_PI / 8., M_PI / 6., M_PI / 12.),
                      Vector3d(1., 2., 3.)),
      RotationMatrixd(RollPitchYawd(M_PI / 4., 5. * M_PI / 6., M_PI / 3)));
}

// Gimbal lock of the orientation of the sphere at pitch = pi/2.
std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereGimbalLock() {
  return GenDistancePairTestSphereSphere(
    RigidTransformd::Identity(),
    RotationMatrixd(RollPitchYawd(M_PI / 6., M_PI_2, M_PI / 6.)));
}

// Two spheres with more general position. Here, we describe the configuration
// in the frame of the first sphere A.  The second sphere B is centered at
// (1,4,8) in A's frame.  We use these identities to design the configuration.
//     0.25^2 + 1^2 + 2^2 = 2.25^2   (1)
//        1^2 + 4^2 + 8^2 = 9^2      (2)
//
//              x x x x x
//           x             x
//         x                 x
//       x                     x
//     x                         x
//    x                           x
//   x                             x
//  x                               x
//  x                               x
//  x                               x
//  x              Bo               x
//  x             /                 x
//  x            /                  x
//  x           /                   x
//   x         /                   x
//    x  ooo  /                   x
//    ox     Ca                  x
//   o   x  /  o               x
//   o  Ao=Cb  o             x
//   o       x o           x
//    o       o x x x x x
//       ooo
//
// From (1), we set A to have radius r_A 2.25 and will have the witness point
// Ca ∈ ∂A at (0.25,1,2) in A's frame.  From (2), we set B to have radius
// r_B 9.0 with the center Bo (1,4,8) in A's frame. The boundary ∂B passes
// through A's center Ao, which is at the same position as the witness point
// Cb ∈ ∂B.  The signed distance between the two spheres equals -2.25, is the
// negative of the radius of A.
//
// @param X_WA the pose of A in world.
// @param R_WB the orientation of B in world.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereSphereNonAligned(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.25);
  auto sphere_B = make_shared<const Sphere>(9.);
  const double radius_A = sphere_A->get_radius();
  // Set up Ca and Bo in A's frame.
  const Vector3d p_ACa(0.25, 1., 2.);
  const Vector3d p_ABo(1., 4., 8.);
  // Set up X_AB and X_BA from the position of B's center p_ABo in A's frame,
  // the given R_WB, and the given X_WA.
  const RotationMatrixd& R_WA = X_WA.rotation();
  const RotationMatrixd R_AW = R_WA.transpose();
  const RotationMatrixd R_AB = R_AW * R_WB;
  const RigidTransformd X_AB(R_AB, p_ABo);
  const RigidTransformd X_BA = X_AB.inverse();
  // Set up X_WB
  const RigidTransformd X_WB = X_WA * X_AB;
  // Set up Cb = Ao.
  const Vector3d p_AAo(0., 0., 0.);
  const Vector3d p_ACb = p_AAo;
  const Vector3d p_BCb = X_BA * p_ACb;

  std::vector<SignedDistancePairTestData> test_data{
      {sphere_A, sphere_B, X_WA, X_WB,
       SignedDistancePair<double>(GeometryId::get_new_id(),
                                  GeometryId::get_new_id(), p_ACa, p_BCb,
                                  -radius_A)}};
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereNonAlignedTransform() {
  return GenDistPairTestSphereSphereNonAligned(
      RigidTransformd(RollPitchYawd(3. * M_PI / 8., 5 * M_PI / 6., M_PI / 12.),
                      Vector3d(1., 2., 3.)),
      RotationMatrixd(RollPitchYawd(M_PI, 5.*M_PI/6., 7*M_PI/12.)));
}

// Sphere-box data for testing ComputeSignedDistancePairwiseClosestPoints.
// We move a small sphere through different configurations:-
// 1. outside the box,
// 2. touching the box,
// 3. slightly overlap the box,
// 4, half inside half outside the box,
// 5. more than half inside the box,
// 6. completely inside and osculating the box, and
// 7. deeply inside the box.
// The sphere's center always stays on the box's positive x-axis, and the
// witness points as expressed in the frames of the sphere and the box stay
// the same in all cases.
//
// @param R_WA specifies the orientation of the sphere A in world.
// @param X_WB specifies the pose of the box B in world.
// @return the test data for testing sphere-box pairwise signed distances.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereBox(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.);
  auto box_B = make_shared<const Box>(16., 12., 8.);
  const double radius = sphere_A->get_radius();
  const double half_x = box_B->size()(0) / 2.;
  struct Configuration {
    Vector3d p_BAo;
    double pair_distance;
  };
  const std::vector<Configuration> configurations{
      // The sphere is outside the box.
      {Vector3d(14., 0., 0.), 4.},
      // The sphere touches the box.
      {Vector3d(10., 0., 0.), 0.},
      // The sphere slightly overlaps the box.
      {Vector3d(9., 0., 0.), -1.},
      // The sphere is half inside and half outside the box.
      {Vector3d(8., 0., 0.), -2.},
      // More than half of the sphere is inside the box.
      {Vector3d(7., 0., 0.), -3.},
      // The sphere is completely inside and osculating the box.
      {Vector3d(6., 0., 0.), -4.},
      // The sphere is deeply inside the box.
      {Vector3d(5., 0., 0.), -5.}};
  const Vector3d p_BCb(half_x, 0., 0.);
  std::vector<SignedDistancePairTestData> test_data;
  for (auto config : configurations) {
    // Set up the pose of A from the translation vector in the configuration
    // and the given rotation parameter R_WA.
    const Vector3d p_WAo = X_WB * config.p_BAo;
    const RigidTransformd X_WA(R_WA, p_WAo);
    // Set up the transformation from B's frame to A's frame.
    const RigidTransformd X_AW = X_WA.inverse();
    const RigidTransformd X_AB = X_AW * X_WB;
    // Calculate Ca in B's frame then change to A's frame.
    const Vector3d p_BCa = config.p_BAo + Vector3d(-radius, 0, 0);
    const Vector3d p_ACa = X_AB * p_BCa;

    test_data.emplace_back(
        sphere_A, box_B, X_WA, X_WB,
        SignedDistancePair<double>(GeometryId::get_new_id(),
                                   GeometryId::get_new_id(), p_ACa, p_BCb,
                                   config.pair_distance));
  }
  return test_data;
}

std::vector<SignedDistancePairTestData> GenDistPairTestSphereBoxTransform() {
  return GenDistPairTestSphereBox(
      RotationMatrixd(RollPitchYawd(M_PI / 8., M_PI / 6., 2. * M_PI / 3)),
      RigidTransformd(RollPitchYawd(3. * M_PI / 8., 5 * M_PI / 6., M_PI / 12.),
                      Vector3d(1., 2., 3.)));
}

// Gimbal lock of the orientation of the sphere at pitch = pi/2.
std::vector<SignedDistancePairTestData>
GenDistPairTestSphereBoxGimbalLock() {
  return GenDistPairTestSphereBox(
      RotationMatrixd(RollPitchYawd(M_PI / 8., M_PI_2, M_PI / 8)),
      RigidTransformd::Identity());
}

std::vector<SignedDistancePairTestData> GenDistPairTestBoxSphere() {
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& sphere_box : GenDistPairTestSphereBox()) {
    test_data.emplace_back(sphere_box.GenSwapAB());
  }
  return test_data;
}

std::vector<SignedDistancePairTestData> GenDistPairTestBoxSphereTransform() {
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& sphere_box : GenDistPairTestSphereBoxTransform()) {
    test_data.emplace_back(sphere_box.GenSwapAB());
  }
  return test_data;
}

// Generates test data for a sphere A with center Ao on the boundary ∂B of a
// box B. The 26 positions of Ao on ∂B can be expressed in B's frame as:-
//   p_BAo ∈ {-h(x),0,+h(x)} x {-h(y),0,+h(y)} x {-h(z),0,+h(z)} - {(0,0,0)},
// where h(x), h(y), and h(z) are the half width, half depth, and half height
// of B respectively. These positions are at the 8 corners, in the middle
// of the 12 edges, and in the middle of the 6 faces of B.
//
// The positions of Ao above is parameterized by the sign vector s expressed
// in B's frame as:
//     s_B = (sx,sy,sz) ∈ {-1,0,+1} x {-1,0,+1} x {-1,0,+1} - {(0,0,0)},
//     p_BAo(s) = (sx * h(x), sy * h(y), sz * h(z)).
//
// For these test cases, the signed distance between A and B is always -r,
// where r is the radius of A. The witness point Cb on ∂B is at Ao.
// The witness point Ca on ∂A is
//     Ca = Ao - r * s/|s|,
// whose position p_ACa in A's frame is expressed as
//     p_ACa = - r * s_A/|s_A|,
// where s_A is the sign vector s expressed in A's frame.
//
// @param R_WA specifies the orientation of the sphere in world.
// @param X_WB specifies the pose of the box in world.
// @return  the test data for testing sphere-box pairwise signed distances.
//
std::vector<SignedDistancePairTestData> GenDistPairTestSphereBoxBoundary(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.);
  auto box_B = make_shared<const Box>(16., 12., 8.);
  const double radius_A = sphere_A->get_radius();
  const Vector3d half_B = box_B->size() / 2.;
  std::vector<SignedDistancePairTestData> test_data;
  // We use sign_x, sign_y, and sign_z to parameterize the positions on the
  // boundary of the box.
  for (const double sign_x : {-1., 0., 1.}) {
    for (const double sign_y : {-1., 0., 1.}) {
      for (const double sign_z : {-1., 0., 1.}) {
        if (sign_x == 0. && sign_y == 0. && sign_z == 0.) continue;
        const Vector3d sign_B(sign_x, sign_y, sign_z);
        // A's center is on ∂B.
        const Vector3d p_BAo = sign_B.array() * half_B.array();
        const Vector3d p_WAo = X_WB * p_BAo;
        const RigidTransformd X_WA(R_WA, p_WAo);
        // The expected witness point Cb on ∂B at A's center.
        const Vector3d p_BCb = p_BAo;
        // Set up the rotation matrix for vectors from B's frame to A's frame.
        const RotationMatrixd& R_WB = X_WB.rotation();
        const RotationMatrixd R_AW = R_WA.transpose();
        const RotationMatrixd R_AB = R_AW * R_WB;
        // Change the expression of the sign vector from sign_B in B's frame
        // to sign_A in A's frame.
        const Vector3d sign_A = R_AB * sign_B;
        // The expected witness point Ca on ∂A expressed in A's frame.
        const Vector3d p_ACa = -radius_A * sign_A.normalized();
        test_data.emplace_back(
            sphere_A, box_B, X_WA, X_WB,
            SignedDistancePair<double>(GeometryId::get_new_id(),
                                       GeometryId::get_new_id(), p_ACa, p_BCb,
                                       -radius_A));
      }
    }
  }
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereBoxBoundaryTransform() {
  return GenDistPairTestSphereBoxBoundary(
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6., M_PI / 3.)),
      RigidTransformd(RollPitchYawd(3. * M_PI / 8., 5 * M_PI / 6., M_PI / 12.),
                      Vector3d(1., 2., 3.)));
}

// Sphere-cylinder data for testing ComputeSignedDistancePairwiseClosestPoints.
// We move a small sphere through different configurations:-
// 1. outside the cylinder,
// 2. touching the cylinder,
// 3. slightly overlap the cylinder,
// 4. half inside half outside the cylinder,
// 5. more than half inside the cylinder,
// 6. completely inside and osculating the cylinder, and
// 7. deeply inside the cylinder.
// The sphere's center always stays on the cylinder's positive z-axis.
// The witness point on the cylinder stays the same in all cases.
// The witness point on the sphere is always the lowest point as expressed
// in the frame of the cylinder.
//
// @param R_WA specifies the orientation of the sphere A in world.
// @param X_WB specifies the pose of the cylinder B in world.
// @return the test data for testing sphere-cylinder pairwise signed distances.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereCylinder(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.);
  auto cylinder_B = make_shared<const Cylinder>(12., 16.);
  const double radius_A = sphere_A->get_radius();
  const double half_length = cylinder_B->get_length() / 2.;
  struct Configuration {
    Vector3d p_BAo;
    double pair_distance;
  };
  const std::vector<Configuration> configurations {
    // The sphere is outside the cylinder.
    {Vector3d(0., 0., 14.), 4.},
    // The sphere touches the cylinder.
    {Vector3d(0., 0., 10.), 0.},
    // The sphere slightly overlaps the cylinder.
    {Vector3d(0., 0., 9.), -1.},
    // The sphere is half inside and half outside the cylinder.
    {Vector3d(0., 0., 8.), -2.},
    // More than half of the sphere is inside the cylinder.
    {Vector3d(0., 0., 7.), -3.},
    // The sphere is completely inside and osculating the cylinder.
    {Vector3d(0., 0., 6.), -4.},
    // The sphere is deeply inside the cylinder.
    {Vector3d(0., 0., 5.), -5.}};
  // Witness point Cb on B.
  const Vector3d p_BCb(0., 0., half_length);
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& config : configurations) {
    // Set up the pose of A from the translation vector in the configuration
    // and the given rotation parameter R_WA.
    const Vector3d p_WAo = X_WB * config.p_BAo;
    const RigidTransformd X_WA(R_WA, p_WAo);
    // Set up the transformation form B's frame to A's frame.
    const RigidTransformd X_AW = X_WA.inverse();
    const RigidTransformd X_AB = X_AW * X_WB;
    // Witness point Ca on A. Calculate its position in B's frame then change
    // to A's frame.
    const Vector3d p_BCa = config.p_BAo + Vector3d(0, 0, -radius_A);
    const Vector3d p_ACa = X_AB * p_BCa;
    test_data.emplace_back(
        sphere_A, cylinder_B, X_WA, X_WB,
        SignedDistancePair<double>(GeometryId::get_new_id(),
                                   GeometryId::get_new_id(), p_ACa, p_BCb,
                                   config.pair_distance));
  }
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereCylinderTransform() {
  return GenDistPairTestSphereCylinder(
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6., M_PI / 3.)),
      RigidTransformd(RollPitchYawd(3. * M_PI / 8., 5 * M_PI / 6., M_PI / 12.),
                      Vector3d(1., 2., 3.)));
}

// Generate test data for a sphere A and cylinder B with center Ao on its
// boundary ∂B.  We use the following mapping to generate the test data from a
// uniform sampling (u,v,w) of the standard cube [-1,1]^3 to p_BAo = (x,y,z)
// on ∂B.
// 0. Generate a uniform sampling (u,v,w) ∈ [-1,1]^3
//       u,v,w ∈ (1/2)*{-2,-1,0,1,2}
//    Skip (u,v,w) with none of |u|,|v|,|w| equals 1.0.  Otherwise, they will
//    map to interior points.
// 1. Map s = (u,v) in the standard square [-1,1]^2 to d = (f,g) in the unit
//    disk centered at (0,0) with radius 1.
//      d = (f,g) = (0,0) if u = v = 0
//                = |u| * s.normalized() otherwise, if |u| >= |v|
//                = |v| * s.normalized() otherwise
// 2. Map d = (f,g) in the unit disk to the circular cross section (x,y) in the
//    cylinder B.
//      (x,y) = r_B * d,  where r_B is the radius of the cylinder B.
// 3. Map w in the standard interval [-1,1] to the vertical coordinate z in
//    the cylinder B.
//      z = w * h, where h is the half length of the cylinder B.
//
// @note: This mapping from (u,v,w) on the cube to (x,y,z) in the cylinder maps
// the top and bottom squares of the cube to the top and bottom caps of the
// cylinder.
//
// In all cases, the signed distance is -r_A, where r_A is the radius of the
// sphere A.

// Since p_BAo = (x,y,z) above is on ∂B, the witness point Cb of the cylinder
// B is always at Ao.
//
// We classify the following cases to compute the witness point Ca on the
// sphere A.  The expression is in B's frame.  The classification is in this
// order:
// 1. (|u|=1 or |v|=1) and |w|=1.
//    Ao is on the circular edges, i.e., the top and bottom circles.
//        p_AoCa_B = -r_A * (f,g,w).normalized()
// 2. |u|=1 or |v|=1.
//    Ao is on the barrel.
//        p_AoCa_B = -r_A * (f,g,0)
// 3. |w|=1.
//    Ao is on the top or bottom caps.
//        p_AoCa_B = -r_A * (0,0,w)
std::vector<SignedDistancePairTestData> GenDistPairTestSphereCylinderBoundary(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(4.);
  auto cylinder_B = make_shared<const Cylinder>(8., 16.);
  const double r_A = sphere_A->get_radius();
  const double r_B = cylinder_B->get_radius();
  const double h = cylinder_B->get_length() / 2.;
  std::vector<SignedDistancePairTestData> test_data;
  for (const double i : {-2., -1., 0., 1., 2.}) {
    for (const double j : {-2., -1., 0., 1., 2.}) {
      for (const double k : {-2., -1., 0., 1., 2.}) {
        // (u,v,w) in the standard cube [-1,1]^3
        const double u = i / 2.;
        const double v = j / 2.;
        const double w = k / 2.;
        const double abs_u = std::abs(u);
        const double abs_v = std::abs(v);
        const double abs_w = std::abs(w);
        // Skip interior points.
        if (abs_u != 1. && abs_v != 1. && abs_w != 1.)
          continue;
        // Map from s(u,v) in square to d(f,g) in the unit disk.
        const Vector2d s(u, v);
        const Vector2d d = (u == 0. && v == 0.) ? Vector2d(0., 0.)
                               : (abs_u >= abs_v) ? abs_u * s.normalized()
                                                  : abs_v * s.normalized();
        // Map from d(f,g) in the unit disk together with w in [-1,1] to
        // (x,y,z) in the cylinder.
        const Vector2d xy = r_B * d;
        const double z = w * h;
        // The position of Ao in B's frame and world frame.
        const Vector3d p_BAo(xy(0), xy(1), z);
        const Vector3d p_WAo = X_WB * p_BAo;
        // The expected witness point Cb on ∂B is at A's center.
        const Vector3d& p_BCb = p_BAo;
        // The pose of the sphere A.
        const RigidTransformd X_WA(R_WA, p_WAo);
        // Calculate rotation matrix R_AB for transforming vector expression
        // from B's frame to A's frame.
        const RotationMatrixd& R_WB = X_WB.rotation();
        const RotationMatrixd& R_AW = R_WA.transpose();
        const RotationMatrixd& R_AB = R_AW * R_WB;
        // Compute the witness point Ca on the sphere A in three cases.  The
        // expression is in B's frame.
        const Vector3d p_AoCa_B =
            ((abs_u == 1. || abs_v == 1.) && abs_w == 1.)
                   // Ao is on the circular edges.
                ? -r_A * Vector3d(d(0), d(1), w).normalized()
                : (abs_u == 1. || abs_v == 1.)
                        // Ao is on the barrel.
                      ? -r_A * Vector3d(d(0), d(1), 0)
                        // Ao is on the top or bottom caps.
                      : -r_A * Vector3d(0., 0., w);
        const Vector3d p_ACa = R_AB * p_AoCa_B;
        // We create new id's for each test case to help distinguish them.
        const GeometryId id_A = GeometryId::get_new_id();
        const GeometryId id_B = GeometryId::get_new_id();
        test_data.emplace_back(
            sphere_A, cylinder_B, X_WA, X_WB,
            SignedDistancePair<double>(id_A, id_B, p_ACa, p_BCb, -r_A));
      }
    }
  }
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereCylinderBoundaryTransform() {
  return GenDistPairTestSphereCylinderBoundary(
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6., M_PI / 3.)),
      RigidTransformd(RollPitchYawd(3. * M_PI / 8., 5 * M_PI / 6., M_PI / 12.),
                      Vector3d(1., 2., 3.)));
}

class SignedDistancePairTest
    : public testing::TestWithParam<SignedDistancePairTestData> {
 public:
  SignedDistancePairTest() {
    auto data = GetParam();
    engine_.AddAnchoredGeometry(*(data.a_), data.X_WA_.GetAsIsometry3(),
                                GeometryIndex(0));
    geometry_map_.push_back(data.expected_result_.id_A);
    engine_.AddDynamicGeometry(*(data.b_), GeometryIndex(1));
    geometry_map_.push_back(data.expected_result_.id_B);
    engine_.UpdateWorldPoses({data.X_WB_.GetAsIsometry3()}, {GeometryIndex(0)});
  }

 protected:
  ProximityEngine<double> engine_;
  std::vector<GeometryId> geometry_map_;

 public:
  // The tolerance value for determining equivalency between expected and
  // tested results. The underlying algorithms have an empirically-determined,
  // hard-coded tolerance of 1e-14 to account for loss of precision due to
  // rigid transformations and this tolerance reflects that.
  static constexpr double tolerance_ = 1e-14;
};

TEST_P(SignedDistancePairTest, SinglePair) {
  const auto& data = GetParam();
  const auto results =
      engine_.ComputeSignedDistancePairwiseClosestPoints(geometry_map_);
  ASSERT_EQ(results.size(), 1);
  const auto& result = results[0];

  EXPECT_NEAR(result.distance, data.expected_result_.distance, tolerance_)
            << "Incorrect signed distance";

  const bool a_then_b = (result.id_A == data.expected_result_.id_A)&&
                        (result.id_B == data.expected_result_.id_B);
  const bool b_then_a = (result.id_B == data.expected_result_.id_A)&&
                        (result.id_A == data.expected_result_.id_B);
  ASSERT_TRUE(a_then_b ^ b_then_a);
  const Vector3d& p_ACa = a_then_b? result.p_ACa : result.p_BCb;
  const Vector3d& p_BCb = a_then_b? result.p_BCb : result.p_ACa;

  EXPECT_TRUE(CompareMatrices(p_ACa, data.expected_result_.p_ACa, tolerance_))
    << "Incorrect witness point.";
  EXPECT_TRUE(CompareMatrices(p_BCb, data.expected_result_.p_BCb, tolerance_))
    << "Incorrect witness point.";

  // Check the invariance that the distance between the two witness points
  // equal the signed distance.
  const Vector3d p_WCb = data.X_WB_ * p_BCb;
  const RigidTransformd X_AW = data.X_WA_.inverse();
  const Vector3d p_ACb = X_AW * p_WCb;
  const double distance_between_witnesses = (p_ACa-p_ACb).norm();
  EXPECT_NEAR(distance_between_witnesses, std::abs(result.distance), tolerance_)
    << "Distance between witness points do not equal the signed distance.";
}

INSTANTIATE_TEST_CASE_P(SphereSphere, SignedDistancePairTest,
    testing::ValuesIn(GenDistancePairTestSphereSphere()));
INSTANTIATE_TEST_CASE_P(SphereSphereTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereTransform()));
INSTANTIATE_TEST_CASE_P(SphereSphereGimbalLock, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereGimbalLock()));

INSTANTIATE_TEST_CASE_P(SphereSphreNonAligned, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereNonAligned()));
INSTANTIATE_TEST_CASE_P(SphereSphreNonAlignedTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereNonAlignedTransform()));

INSTANTIATE_TEST_CASE_P(SphereBox, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBox()));
INSTANTIATE_TEST_CASE_P(SphereBoxTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBoxTransform()));
INSTANTIATE_TEST_CASE_P(SphereBoxGimbalLock, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBoxGimbalLock()));

INSTANTIATE_TEST_CASE_P(BoxSphere, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestBoxSphere()));
INSTANTIATE_TEST_CASE_P(BoxSphereTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestBoxSphereTransform()));

INSTANTIATE_TEST_CASE_P(SphereBoxBoundary, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBoxBoundary()));
INSTANTIATE_TEST_CASE_P(SphereBoxBoundaryTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBoxBoundaryTransform()));

INSTANTIATE_TEST_CASE_P(SphereCylinder, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCylinder()));
INSTANTIATE_TEST_CASE_P(SphereCylinderTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCylinderTransform()));
INSTANTIATE_TEST_CASE_P(SphereCylinderBoundary, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCylinderBoundary()));
INSTANTIATE_TEST_CASE_P(SphereCylinderBoundaryTransform,
    SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCylinderBoundaryTransform()));

// This tests that signed distance queries against a halfspace throw an
// intelligible exception rather than a segfault.
GTEST_TEST(SignedDistancePairError, HalfspaceException) {
  // Note: this doesn't fully test the condition for emitting the error; we
  // have no *real* control over which geometry is the first geometry and
  // second geometry in the pair being tested. However, this test isn't
  // intended to be long-lasting; we want to correct FCL's short-coming soon
  // so that the behavior (and this test) can be removed.
  ProximityEngine<double> engine;
  std::vector<GeometryId> geometry_map;

  Sphere sphere{0.5};
  engine.AddDynamicGeometry(sphere, GeometryIndex(0));
  geometry_map.push_back(GeometryId::get_new_id());

  HalfSpace halfspace;
  engine.AddAnchoredGeometry(halfspace, Isometry3d::Identity(),
                             GeometryIndex(1));
  geometry_map.push_back(GeometryId::get_new_id());

  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.ComputeSignedDistancePairwiseClosestPoints(geometry_map),
      std::logic_error,
      "Signed distance .* halfspaces .* not .* supported.* Try .* box .*");
}

// Concentric geometries A, B do not have a unique pair of witness points
// Na, Nb. We inherit another test fixture from SignedDistancePairTest, so we
// can define another TEST_P that checks |Na-Nb| = -signed_distance but does
// not check the locations of Na and Nb individually.
class SignedDistancePairConcentricTest : public SignedDistancePairTest {};

TEST_P(SignedDistancePairConcentricTest, DistanceInvariance) {
  const auto& data = GetParam();
  const auto results =
      engine_.ComputeSignedDistancePairwiseClosestPoints(geometry_map_);
  ASSERT_EQ(results.size(), 1);
  const auto& result = results[0];

  EXPECT_NEAR(result.distance, data.expected_result_.distance, tolerance_)
    << "Incorrect signed distance";

  const bool a_then_b = (result.id_A == data.expected_result_.id_A)&&
      (result.id_B == data.expected_result_.id_B);
  const bool b_then_a = (result.id_B == data.expected_result_.id_A)&&
      (result.id_A == data.expected_result_.id_B);
  ASSERT_TRUE(a_then_b ^ b_then_a);
  const Vector3d& p_ACa = a_then_b? result.p_ACa : result.p_BCb;
  const Vector3d& p_BCb = a_then_b? result.p_BCb : result.p_ACa;

  // Check the invariance that the distance between the two witness points
  // equal the signed distance.
  const Vector3d p_WCb = data.X_WB_ * p_BCb;
  const RigidTransformd X_AW = data.X_WA_.inverse();
  const Vector3d p_ACb = X_AW * p_WCb;
  const double distance_between_witnesses = (p_ACa-p_ACb).norm();
  EXPECT_NEAR(distance_between_witnesses, -result.distance, tolerance_)
    << "Incorrect distance between witness points.";
}

// Generates one record of test data for two spheres whose centers are at the
// same point.
// @param radius_A specifies the radius of the first sphere A.
// @param radius_B specifies the radius of the second sphere B.
// @param X_WA specifies the pose of A in world.
// @param R_WB specifies the orientation of B in world.
SignedDistancePairTestData
GenDistPairTestTwoSpheresConcentricBasic(
    const double radius_A = 1.0,
    const double radius_B = 1.0,
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(radius_A);
  auto sphere_B = make_shared<const Sphere>(radius_B);
  const RigidTransformd X_WB(R_WB, X_WA.translation());
  const RigidTransformd X_AW = X_WA.inverse();
  const RigidTransformd X_AB = X_AW * X_WB;

  // Since the two spheres are concentric, we arbitrarily pick the witness
  // point Cb at (radius_B,0,0) in B's frame and calculate the corresponding
  // witness point Ca in A's frame, but the TEST_P will ignore them.
  const Vector3d p_BCb(radius_B, 0.0, 0.0);
  const Vector3d p_ACb = X_AB * p_BCb;
  const Vector3d p_ACa = -(radius_A / radius_B) * p_ACb;

  return  SignedDistancePairTestData(
      sphere_A, sphere_B, X_WA, X_WB,
       SignedDistancePair<double>(
           GeometryId::get_new_id(), GeometryId::get_new_id(),
           p_ACa, p_BCb, -radius_A - radius_B));
}

// Generates test data for two spheres A and B whose centers are at the same
// point with varying radii.
// @param X_WA specifies the pose of A in world.
// @param R_WB specifies the orientation of B in world.
std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereConcentric(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  std::vector<SignedDistancePairTestData> test_data{
      // Both unit spheres.
      GenDistPairTestTwoSpheresConcentricBasic(1.0, 1.0, X_WA, R_WB),
      // One is smaller than a unit sphere, another is larger than a unit
      // sphere.
      GenDistPairTestTwoSpheresConcentricBasic(0.5, 2.0, X_WA, R_WB),
      // Different sizes. Both spheres are smaller than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(0.3, 0.7, X_WA, R_WB),
      // Same sizes. Both spheres are smaller than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(0.7, 0.7, X_WA, R_WB),
      // Different sizes. Both spheres are larger than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(3.0, 7.0, X_WA, R_WB),
      // Same sizes. Both spheres are larger than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(7.0, 7.0, X_WA, R_WB),
  };
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereConcentricTransform() {
  return GenDistPairTestSphereSphereConcentric(
    RigidTransformd(RollPitchYawd(M_PI_4, M_PI_2, M_PI),
                    Vector3d(1., 2., 3.)),
    RotationMatrixd(RollPitchYawd(M_PI, M_PI/6., M_PI)));
}

// Gimbal lock of the orientation of the sphere at pitch = pi/2.
std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereConcentricGimbalLock() {
  return GenDistPairTestSphereSphereConcentric(
    RigidTransformd::Identity(),
    RotationMatrixd(RollPitchYawd(M_PI, M_PI_2, M_PI)));
}

// Generates test data for a sphere and a box with the same centers.
// @param X_WA specifies the pose of the sphere A in world.
// @param R_WB specifies the orientation of the box B in world.
std::vector<SignedDistancePairTestData>
GenDistPairTestSphereBoxConcentric(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.);
  auto box_B = make_shared<const Box>(4., 8., 16.);
  const double radius_A = sphere_A->get_radius();
  const Vector3d half_B = box_B->size() / 2.;
  const RigidTransformd X_WB(R_WB, X_WA.translation());
  const RigidTransformd X_AW = X_WA.inverse();
  const RigidTransformd X_AB = X_AW * X_WB;

  // Since A and B are concentric, we arbitrarily pick the witness point Cb
  // at (half_B.x, 0, 0) in B's frame and calculate the corresponding witness
  // point Ca in A's frame, but the TEST_P will ignore them.
  const Vector3d p_BCb(half_B(0), 0., 0.);
  const Vector3d p_ACb = X_AB * p_BCb;
  const Vector3d p_ACa = -(radius_A / half_B(0)) * p_ACb;

  std::vector<SignedDistancePairTestData> test_data{
      {sphere_A, box_B, X_WA, X_WB,
       SignedDistancePair<double>(
           GeometryId::get_new_id(), GeometryId::get_new_id(),
           p_ACa, p_BCb, -radius_A - half_B(0))}
  };
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereBoxConcentricTransform() {
  return GenDistPairTestSphereBoxConcentric(
      RigidTransformd(RollPitchYawd(M_PI_4, M_PI_2, M_PI),
                      Vector3d(1., 2., 3.)),
      RotationMatrixd(RollPitchYawd(M_PI, M_PI/6., M_PI)));
}

INSTANTIATE_TEST_CASE_P(SphereSphereConcentric,
    SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereSphereConcentric()));
INSTANTIATE_TEST_CASE_P(SphereSphereConcentricTransform,
    SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereSphereConcentricTransform()));
INSTANTIATE_TEST_CASE_P(SphereSphereConcentricGimbalLock,
    SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereSphereConcentricGimbalLock()));

INSTANTIATE_TEST_CASE_P(SphereBoxConcentric,
    SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereBoxConcentric()));
INSTANTIATE_TEST_CASE_P(SphereBoxConcentricTransform,
    SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereBoxConcentricTransform()));

//
// Tests the repeatability of the call to compute the signed distance between
// closest pairs of dynamic objects when the objects keep the same poses.
// Related to https://github.com/RobotLocomotion/drake/issues/10286
//
// TODO(DamrongGuoy): Remove this test after FCL guarantees consistency in
// the result of broadphase distance. See this FCL issue:
// https://github.com/flexible-collision-library/fcl/issues/368
//
// We use value-parameterized tests to generate data on many kinds of
// pairs of shapes. We separate the code to generate test data from the test
// fixture and the test procedure for modularity.
//
// The test is organized in the following way:
// 1. Define test data SignedDistancePairRepeatTestData.
// 2. Define test fixture SignedDistancePairRepeatabilityTest that takes a
//    parameter of type SignedDistancePairRepeatTestData.
// 3. Define test procedure TEST_P() of the test fixture.
// 4. Instantiate the tests from the test data and the test fixture.
//

// Test data.
struct SignedDistancePairRepeatTestData {
  SignedDistancePairRepeatTestData(const shared_ptr<const Shape>& a,
                                   const shared_ptr<const Shape>& b,
                                   const RigidTransformd& X_WA_in,
                                   const RigidTransformd& X_WB_in)
      : shape_A(a), shape_B(b), X_WA(X_WA_in), X_WB(X_WB_in) {}

  shared_ptr<const Shape> shape_A;
  shared_ptr<const Shape> shape_B;
  RigidTransformd X_WA;
  RigidTransformd X_WB;
};

std::vector<SignedDistancePairRepeatTestData>
GenDistPairRepeatabilityTestData() {
  std::vector<shared_ptr<const Shape>> shapes_A{
      make_shared<const Sphere>(0.1),
      make_shared<const Box>(0.1, 0.2, 0.3),
      make_shared<const Cylinder>(0.1, 3.1),
      make_shared<const Convex>(
          drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
          1.0)};
  std::vector<shared_ptr<const Shape>> shapes_B{
      make_shared<const Sphere>(0.2),
      make_shared<const Box>(0.11, 0.21, 0.31),
      make_shared<const Cylinder>(0.2, 3.2),
      make_shared<const Convex>(
          drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
          1.1)};
  const Translation3d X_WA(0.1, 0.2, 0.3);
  const Translation3d X_WB(0.11, 0.21, 0.31);
  std::vector<SignedDistancePairRepeatTestData> test_data;
  for (shared_ptr<const Shape>& a : shapes_A)
    for (shared_ptr<const Shape>& b : shapes_B)
      test_data.emplace_back(a, b, X_WA, X_WB);
  return test_data;
}

// Test fixture.
class SignedDistancePairRepeatabilityTest
    : public testing::TestWithParam<SignedDistancePairRepeatTestData> {
 public:
  SignedDistancePairRepeatabilityTest() {
    auto data = GetParam();
    engine_.AddDynamicGeometry(*(data.shape_A), GeometryIndex(0));
    engine_.AddDynamicGeometry(*(data.shape_B), GeometryIndex(1));
    geometry_map_.push_back(GeometryId::get_new_id());
    geometry_map_.push_back(GeometryId::get_new_id());
    X_WG_.push_back(data.X_WA.GetAsIsometry3());
    X_WG_.push_back(data.X_WB.GetAsIsometry3());
    geometry_indices_.push_back(GeometryIndex(0));
    geometry_indices_.push_back(GeometryIndex(1));
  }

 protected:
  ProximityEngine<double> engine_;
  std::vector<GeometryId> geometry_map_;
  std::vector<Isometry3d> X_WG_;
  std::vector<GeometryIndex> geometry_indices_;
};

// Test procedure.
// TODO(DamrongGuoy): Use a more direct way to test that the function
// drake::geometry::internal::<unnamed>::DistanceCallback() always passes two
// objects A and B in a consistent order by their geometry id.
TEST_P(SignedDistancePairRepeatabilityTest, SinglePair) {
  engine_.UpdateWorldPoses(X_WG_, geometry_indices_);
  const auto results =
      engine_.ComputeSignedDistancePairwiseClosestPoints(geometry_map_);
  ASSERT_EQ(results.size(), 1);
  const double signed_distance_first_call = results[0].distance;

  // Repeat the computation many times and make sure we always get exactly
  // the same signed distance as the first one above. No tolerance.
  const int kRepeat = 7;
  for (int count = 1; count <= kRepeat; ++count) {
    engine_.UpdateWorldPoses(X_WG_, geometry_indices_);
    const auto repeat_results =
        engine_.ComputeSignedDistancePairwiseClosestPoints(geometry_map_);
    ASSERT_EQ(repeat_results.size(), 1);
    // Compare using all the bits of `double`.
    ASSERT_EQ(signed_distance_first_call, repeat_results[0].distance)
        << "Repeated call did not give exactly the same signed distance.";
  }
}

// Instantiate the tests.
INSTANTIATE_TEST_CASE_P(RepeatabilityTest, SignedDistancePairRepeatabilityTest,
                        testing::ValuesIn(GenDistPairRepeatabilityTestData()));


// Given a sphere S and box B. The box's height and depth are large (much larger
// than the diameter of the sphere), but the box's *width* is *less* than the
// sphere diameter. The sphere will contact it such that it's penetration is
// along the "width" axis.
//
// The sphere will start in a non-colliding state and subsequently move in the
// width direction. As long as the sphere's center is on the same side of the
// box's "origin", the contact normal direction should remain unchanged. But as
// soon as it crosses the origin, the contact normal should flip directions.
//
// Non-colliding state - no collision reported.
//       y
//      ┇ │ ┇       ←  movement of sphere
//      ┃ │ ┃       o o
//      ┃ │ ┃    o       o
//      ┃ │ ┃   o         o
// ─────╂─┼─╂───o────c────o───────── x
//      ┃ │ ┃   o         o
//      ┃ │ ┃    o       o
//      ┃ │ ┃       o o            `c' marks the sphere center
//      ┇ │ ┇
//
// Sphere's center is just to the right of the box's origin.
// From the first contact up to this point, the contact normal should point to
// the left (into the box). The direction is based on how PenetrationAsPointPair
// defines the normal direction -- from geometry B into geometry A. Based on
// how the geometries are defined, this means from the sphere, into the box.
//       y
//      ┇ │ ┇       ←  movement of sphere
//      ┃ o o
//     o┃ │ ┃  o
//    o ┃ │ ┃   o
// ───o─╂─┼c╂───o─────────────── x
//    o ┃ │ ┃   o
//     o┃ │ ┃  o
//      ┃ o o
//      ┇ │ ┇
//
// Discontinuous colliding state - crossed the origin; contact normal flips to
// the right.
//       y
//        ┇ │ ┇       ←  movement of sphere
//        o o ┃
//     o  ┃ │ ┃o
//    o   ┃ │ ┃ o
// ───o───╂c┼─╂─o─────────────── x
//    o   ┃ │ ┃ o
//     o  ┃ │ ┃o
//        o o ┃
//        ┇ │ ┇
//
// Note: this test is in direct response to a very *particular* observed bug
// in simulation which pointed to errors in FCL. This is provided as a
// regression test once FCL is patched. The original failure condition has been
// reproduced with this simplified version of the original problem. Note:
// passing this test does not guarantee general correctness of box-sphere
// collision; only that this particular bug is gone.

// Supporting structure for the query.
struct SpherePunchData {
  const std::string description;
  const Vector3d sphere_pose;
  const int contact_count;
  // This is the contact normal pointing *into* the box.
  const Vector3d contact_normal;
  const double depth;
};

GTEST_TEST(ProximityEngineCollisionTest, SpherePunchThroughBox) {
  ProximityEngine<double> engine;
  const double radius = 0.5;
  const double w = radius;        // Box width smaller than diameter.
  const double half_w = w / 2;
  const double h = 10 * radius;   // Box height much larger than sphere.
  const double d = 10 * radius;   // Box depth much larger than sphere.
  const double eps = std::numeric_limits<double>::epsilon();
  GeometryIndex box_index(0);
  engine.AddDynamicGeometry(Box{w, h, d}, box_index);
  GeometryIndex sphere_index(1);
  engine.AddDynamicGeometry(Sphere{radius}, sphere_index);

  GeometryId box_id = GeometryId::get_new_id();
  GeometryId sphere_id = GeometryId::get_new_id();
  std::vector<GeometryId> geometry_map{box_id, sphere_id};

  std::vector<Isometry3d> poses{Isometry3d::Identity(), Isometry3d::Identity()};
  // clang-format off
  std::vector<SpherePunchData> test_data{
      // In non-penetration, contact_normal and depth values don't matter; they
      // are not tested.
      {"non-penetration",
       {radius + half_w + 0.1, 0, 0}, 0, {0, 0, 0}, -1.},
      {"shallow penetration -- sphere center outside of box",
       {radius + 0.75 * half_w, 0, 0}, 1, {-1, 0, 0}, 0.25 * half_w},
      {"deep penetration -- sphere contacts opposite side of the box",
       {radius - half_w, 0, 0}, 1, {-1, 0, 0}, w},
      {"sphere's origin is just to the right of the box center",
       {eps, 0, 0}, 1, {-1, 0, 0}, radius + half_w - eps},
      {"sphere's center has crossed the box's origin - flipped normal",
       {-eps, 0, 0}, 1, {1, 0, 0}, radius + half_w - eps}};
  // clang-format on
  std::vector<GeometryIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), GeometryIndex(0));
  for (const auto& test : test_data) {
    poses[1].translation() = test.sphere_pose;
    engine.UpdateWorldPoses(poses, indices);
    std::vector<PenetrationAsPointPair<double>> results =
        engine.ComputePointPairPenetration(geometry_map);

    ASSERT_EQ(static_cast<int>(results.size()), test.contact_count)
        << "Failed for the " << test.description << " case";
    if (test.contact_count == 1) {
      const PenetrationAsPointPair<double>& penetration = results[0];
      // Normal direction is predicated on the sphere being geometry B.
      ASSERT_EQ(penetration.id_A, box_id);
      ASSERT_EQ(penetration.id_B, sphere_id);
      EXPECT_TRUE(CompareMatrices(penetration.nhat_BA_W, test.contact_normal,
                                  eps, MatrixCompareType::absolute))
                << "Failed for the " << test.description << " case";
      // For this simple, axis-aligned test (where all the values are nicely
      // powers of 2), I should expect perfect answers.
      EXPECT_EQ(penetration.depth - test.depth, 0)
                << "Failed for the " << test.description << " case";
    }
  }
}

// Robust Box-Primitive tests. Tests collision of the box with other primitives
// in a uniform framework. These tests parallel tests located in fcl.

// This performs a very specific test. It collides a rotated box with a
// surface that is tangent to the z = 0 plane. The box is a cube with unit size.
// The goal is to transform the box such that:
//   1. the corner of the box C, located at p_BoC_B = (-0.5, -0.5, -0.5),
//      transformed by the box's pose X_WB, ends up at the position
//      p_WC = (0, 0, -kDepth), i.e., p_WC = X_WB * p_BoC_B, and
//   2. all other corners are transformed to lie above the z = 0 plane.
//
// In this configuration, the corner C becomes the unique point of deepest
// penetration in the interior of the half space.
//
// It is approximately *this* picture
//        ┆  ╱╲
//        ┆ ╱  ╲
//        ┆╱    ╲
//       ╱┆╲    ╱
//      ╱ ┆ ╲  ╱
//     ╱  ┆  ╲╱
//     ╲  ┆  ╱
//      ╲ ┆ ╱
//  _____╲┆╱_____    ╱____ With small penetration depth of d
//  ░░░░░░┆░░░░░░    ╲
//  ░░░░░░┆░░░░░░
//  ░░░Tangent░░░
//  ░░░░shape░░░░
//  ░░interior░░░
//  ░░░░░░┆░░░░░░
//
// We can use this against various *convex* shapes to determine uniformity of
// behavior. As long as the convex tangent shape *touches* the z = 0 plane at
// (0, 0, 0), and the shape is *large* compared to the penetration depth, then
// we should get a fixed, known contact. Specifically, if we assume the tangent
// shape is A and the box is B, then
//   - the normal is (0, 0, -1) from box into the plane,
//   - the penetration depth is the specified depth used to configure the
//     position of the box, and
//   - the contact position is (0, 0, -depth / 2).
//
// Every convex shape type can be used as the tangent shape as follows:
//   - plane: simply define the z = 0 plane.
//   - box: define a box whose top face lies on the z = 0 and encloses the
//     origin.
//   - sphere: place the sphere at (0, 0 -radius).
//   - cylinder: There are two valid configurations (radius & length >> depth).
//     - Place a "standing" cylinder at (0, 0, -length/2).
//     - Rotate the cylinder so that its length axis is parallel with the z = 0
//       plane and the displace downward (0, 0, -radius). Described as "prone".
//
// Note: there are an infinite number of orientations that satisfy the
// configuration described above. They should *all* provide the same collision
// results. We provide two representative orientations to illustrate issues
// with the underlying functionality -- the *quality* of the answer depends on
// the orientation of the box. When the problem listed in Drake issue 7656 is
// resolved, all tests should resolve to the same answer with the same tight
// precision.
// TODO(SeanCurtis-TRI): Add other shapes as they become available.
class BoxPenetrationTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Confirm all of the constants are consistent (in case someone tweaks them
    // later). In this case, tangent geometry must be much larger than the
    // depth.
    EXPECT_GT(kRadius, kDepth * 100);
    EXPECT_GT(kLength, kDepth * 100);

    // Confirm that the poses of the box satisfy the conditions described above.
    for (auto func : {X_WB_1, X_WB_2}) {
      const math::RigidTransformd X_WB = func(p_BoC_B_, p_WC_);
      // Confirm that p_BoC_B transforms to p_WC.
      const Vector3d p_WC_test = X_WB * p_BoC_B_;
      EXPECT_TRUE(CompareMatrices(p_WC_test, p_WC_, 1e-15,
                                  MatrixCompareType::absolute));

      // Confirm that *all* other points map to values where z > 0.
      for (double x : {-0.5, 0.5}) {
        for (double y : {-0.5, 0.5}) {
          for (double z : {-0.5, 0.5}) {
            const Vector3d p_BoC_B{x, y, z};
            if (p_BoC_B.isApprox(p_BoC_B_)) continue;
            const Vector3d p_WC = X_WB * p_BoC_B;
            EXPECT_GT(p_WC(2), 0);
          }
        }
      }
    }

    // Configure the expected penetration characterization.
    expected_penetration_.p_WCa << 0, 0, 0;       // Tangent plane
    expected_penetration_.p_WCb = p_WC_;          // Cube
    expected_penetration_.nhat_BA_W << 0, 0, -1;  // From cube into plane
    expected_penetration_.depth = kDepth;
    // NOTE: The ids are set by the individual calling tests.
  }

  enum TangentShape {
    TangentPlane,
    TangentSphere,
    TangentBox,
    TangentStandingCylinder,
    TangentProneCylinder,
    TangentConvex
  };

  // The test that produces *bad* results based on the box orientation. Not
  // called "bad" because when FCL is fixed, they'll both be good.
  void TestCollision1(TangentShape shape_type, double tolerance) {
    TestCollision(shape_type, tolerance, X_WB_1(p_BoC_B_, p_WC_));
  }

  // The test that produces *good* results based on the box orientation. Not
  // called "good" because when FCL is fixed, they'll both be good.
  void TestCollision2(TangentShape shape_type, double tolerance) {
    TestCollision(shape_type, tolerance, X_WB_2(p_BoC_B_, p_WC_));
  }

 private:
  // Perform the collision test against the indicated shape and confirm the
  // results to the given tolerance.
  void TestCollision(TangentShape shape_type, double tolerance,
                     const math::RigidTransformd& X_WB) {
    GeometryIndex tangent_index(0);
    engine_.AddDynamicGeometry(shape(shape_type), tangent_index);
    GeometryId tangent_id = GeometryId::get_new_id();
    geometry_map_.push_back(tangent_id);

    GeometryIndex box_index(1);
    engine_.AddDynamicGeometry(box_, box_index);
    GeometryId box_id = GeometryId::get_new_id();
    geometry_map_.push_back(box_id);

    // Confirm that there are no other geometries interfering.
    EXPECT_EQ(tangent_index, 0);
    EXPECT_EQ(box_index, 1);
    ASSERT_EQ(engine_.num_dynamic(), 2);

    // Update the poses of the geometry.
    std::vector<Isometry3d> poses{shape_pose(shape_type),
                                  X_WB.GetAsIsometry3()};
    std::vector<GeometryIndex> indices(poses.size());
    std::iota(indices.begin(), indices.end(), GeometryIndex(0));
    engine_.UpdateWorldPoses(poses, indices);
    std::vector<PenetrationAsPointPair<double>> results =
        engine_.ComputePointPairPenetration(geometry_map_);

    ASSERT_EQ(results.size(), 1u) << "Against tangent "
                                  << shape_name(shape_type);

    const PenetrationAsPointPair<double>& contact = results[0];
    Vector3d normal;
    Vector3d p_Ac;
    Vector3d p_Bc;
    if (contact.id_A == tangent_id && contact.id_B == box_id) {
      // The documented encoding of expected_penetration_.
      normal = expected_penetration_.nhat_BA_W;
      p_Ac = expected_penetration_.p_WCa;
      p_Bc = expected_penetration_.p_WCb;
    } else if (contact.id_A == box_id && contact.id_B == tangent_id) {
      // The reversed encoding of expected_penetration_.
      normal = -expected_penetration_.nhat_BA_W;
      p_Ac = expected_penetration_.p_WCb;
      p_Bc = expected_penetration_.p_WCa;
    } else {
      GTEST_FAIL() << "Wrong geometry ids reported in contact for tangent "
                   << shape_name(shape_type) << ". Expected " << tangent_id
                   << " and " << box_id << ". Got " << contact.id_A << " and "
                   << contact.id_B;
    }
    EXPECT_TRUE(CompareMatrices(contact.nhat_BA_W, normal, tolerance))
        << "Against tangent " << shape_name(shape_type);
    EXPECT_TRUE(CompareMatrices(contact.p_WCa, p_Ac, tolerance))
        << "Against tangent " << shape_name(shape_type);
    EXPECT_TRUE(CompareMatrices(contact.p_WCb, p_Bc, tolerance))
        << "Against tangent " << shape_name(shape_type);
    EXPECT_NEAR(contact.depth, expected_penetration_.depth, tolerance)
        << "Against tangent " << shape_name(shape_type);
  }

  // The expected collision result -- assumes that A is the tangent object and
  // B is the colliding box.
  PenetrationAsPointPair<double> expected_penetration_;

  // Produces the X_WB that produces high-quality answers.
  static math::RigidTransformd X_WB_2(const Vector3d& p_BoC_B,
                                      const Vector3d& p_WC) {
    // Compute the pose of the colliding box.
    // a. Orient the box so that the corner p_BoC_B = (-0.5, -0.5, -0.5) lies in
    //    the most -z extent. With only rotation, p_BoC_B != p_WC.
    const math::RotationMatrixd R_WB(AngleAxisd(std::atan(M_SQRT2),
                                     Vector3d(M_SQRT1_2, -M_SQRT1_2, 0)));

    // b. Translate it so that the rotated corner p_BoC_W lies at
    //    (0, 0, -d).
    const Vector3d p_BoC_W = R_WB * p_BoC_B;
    const Vector3d p_WB = p_WC - p_BoC_W;
    return math::RigidTransformd(R_WB, p_WB);
  }

  // Produces the X_WB that produces low-quality answers.
  static math::RigidTransformd X_WB_1(const Vector3d& p_BoC_B,
                                      const Vector3d& p_WC) {
    // Compute the pose of the colliding box.
    // a. Orient the box so that the corner p_BoC_B = (-0.5, -0.5, -0.5) lies in
    //    the most -z extent. With only rotation, p_BoC_B != p_WC.
    const math::RotationMatrixd R_WB(AngleAxisd(-M_PI_4, Vector3d::UnitY()) *
                                     AngleAxisd(M_PI_4, Vector3d::UnitX()));

    // b. Translate it so that the rotated corner p_BoC_W lies at
    //    (0, 0, -d).
    const Vector3d p_BoC_W = R_WB * p_BoC_B;
    const Vector3d p_WB = p_WC - p_BoC_W;
    return math::RigidTransformd(R_WB, p_WB);
  }

  // Map enumeration to string for error messages.
  static const char* shape_name(TangentShape shape) {
    switch (shape) {
      case TangentPlane:
        return "plane";
      case TangentSphere:
        return "sphere";
      case TangentBox:
        return "box";
      case TangentStandingCylinder:
        return "standing cylinder";
      case TangentProneCylinder:
        return "prone cylinder";
      case TangentConvex:
        return "convex";
    }
    return "undefined shape";
  }

  // Map enumeration to the configured shapes.
  const Shape& shape(TangentShape shape) {
    switch (shape) {
      case TangentPlane:
        return tangent_plane_;
      case TangentSphere:
        return tangent_sphere_;
      case TangentBox:
        return tangent_box_;
      case TangentStandingCylinder:
      case TangentProneCylinder:
        return tangent_cylinder_;
      case TangentConvex:
        return tangent_convex_;
    }
    // GCC considers this function ill-formed - no apparent return value. This
    // exception alleviates its concern.
    throw std::logic_error(
        "Trying to acquire shape for unknown shape enumerated value: " +
        std::to_string(shape));
  }

  // Map enumeration to tangent pose.
  Isometry3d shape_pose(TangentShape shape) {
    Isometry3d pose = Isometry3d::Identity();
    switch (shape) {
      case TangentPlane:
        break;  // leave it at the identity
      case TangentSphere:
        pose.translation() = Vector3d{0, 0, -kRadius};
        break;
      case TangentBox:
      // The tangent convex is a cube of the same size as the tangent box.
      // That is why we give them the same pose.
      case TangentConvex:
      case TangentStandingCylinder:
        pose.translation() = Vector3d{0, 0, -kLength / 2};
        break;
      case TangentProneCylinder:
        pose = AngleAxisd{M_PI_2, Vector3d::UnitX()};
        pose.translation() = Vector3d{0, 0, -kRadius};
        break;
    }
    return pose;
  }

  // Test constants. Geometric measures must be much larger than depth. The test
  // enforces a ratio of at least 100. Using these in a GTEST precludes the
  // possibility of being constexpr initialized; GTEST takes references.
  static const double kDepth;
  static const double kRadius;
  static const double kLength;

  ProximityEngine<double> engine_;
  std::vector<GeometryId> geometry_map_;

  // The various geometries used in the collision test.
  const Box box_{1, 1, 1};
  const Sphere tangent_sphere_{kRadius};
  const Box tangent_box_{kLength, kLength, kLength};
  const HalfSpace tangent_plane_;  // Default construct the z = 0 plane.
  const Cylinder tangent_cylinder_{kRadius, kLength};
  // We scale the convex shape by 5.0 to match the tangent_box_ of size 10.0.
  // The file "quad_cube.obj" contains the cube of size 2.0.
  const Convex tangent_convex_{drake::FindResourceOrThrow(
      "drake/geometry/test/quad_cube.obj"), 5.0};

  const Vector3d p_WC_{0, 0, -kDepth};
  const Vector3d p_BoC_B_{-0.5, -0.5, -0.5};
};

// See documentation. All geometry constants must be >= kDepth * 100.
const double BoxPenetrationTest::kDepth = 1e-3;
const double BoxPenetrationTest::kRadius = 1.0;
const double BoxPenetrationTest::kLength = 10.0;

TEST_F(BoxPenetrationTest, TangentPlane1) {
  TestCollision1(TangentPlane, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentPlane2) {
  TestCollision2(TangentPlane, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentBox1) { TestCollision1(TangentBox, 1e-12); }

TEST_F(BoxPenetrationTest, TangentBox2) { TestCollision2(TangentBox, 1e-12); }

TEST_F(BoxPenetrationTest, TangentSphere1) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. Largely due to normal
  // calculation. See related Drake issue 7656.
  TestCollision1(TangentSphere, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentSphere2) {
  TestCollision2(TangentSphere, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentStandingCylinder1) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. See related Drake issue 7656.
  TestCollision1(TangentStandingCylinder, 1e-3);
}

TEST_F(BoxPenetrationTest, TangentStandingCylinder2) {
  TestCollision2(TangentStandingCylinder, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentProneCylinder1) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. Largely due to normal
  // calculation. See related Drake issue 7656.
  TestCollision1(TangentProneCylinder, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentProneCylinder2) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. In this case, the largest error
  // is in the position of the contact points. See related Drake issue 7656.
  TestCollision2(TangentProneCylinder, 1e-4);
}

TEST_F(BoxPenetrationTest, TangentConvex1) {
  // TODO(DamrongGuoy): We should check why we cannot use a smaller tolerance.
  TestCollision1(TangentConvex, 1e-3);
}

TEST_F(BoxPenetrationTest, TangentConvex2) {
  // TODO(DamrongGuoy): We should check why we cannot use a smaller tolerance.
  TestCollision2(TangentConvex, 1e-3);
}

// Attempting to add a dynamic Mesh should cause an abort.
GTEST_TEST(ProximityEngineTests, AddDynamicMesh) {
  ProximityEngine<double> engine;
  Mesh mesh{"invalid/path/thing.obj", 1.0};
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.AddDynamicGeometry(mesh, GeometryIndex(0)),
      std::exception,
      ".*The proximity engine does not support meshes yet.*");
}

// Attempting to add a anchored Mesh should cause an abort.
GTEST_TEST(ProximityEngineTests, AddAnchoredMesh) {
  ProximityEngine<double> engine;
  Mesh mesh{"invalid/path/thing.obj", 1.0};
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.AddAnchoredGeometry(mesh, Isometry3d::Identity(),
                                 GeometryIndex(0)),
      std::exception,
      ".*The proximity engine does not support meshes yet.*");
}

// This is a one-off test. Exposed in issue #10577. A point penetration pair
// was returned for a zero-depth contact. This reproduces the geometry that
// manifested the error. The reproduction isn't *exact*; it's been simplified to
// a simpler configuration. Specifically, the important characteristics are:
//   - both box and cylinder are ill aspected (one dimension is several orders
//     of magnitude smaller than the other two),
//   - the cylinder is placed away from the center of the box face,
//   - the box is rotated 90 degrees around it's z-axis -- note swapping box
//     dimensions with an identity rotation did *not* produce equivalent
//     results, and
//   - FCL uses GJK/EPA to solve box-cylinder collision (this is beyond control
//     of this test).
//
// Libccd upgraded how it handles degenerate simplices. The upshot of that is
// FCL would still return the same penetration depth, but instead of returning
// a gibberish normal, it returns a zero vector. We want to make sure we don't
// report zero-penetration as penetration, even in these numerically,
// ill-conditioned scenarios. So, we address it up to a tolerance.
GTEST_TEST(ProximityEngineTests, Issue10577Regression_Osculation) {
  ProximityEngine<double> engine;
  GeometryId id_A = GeometryId::get_new_id();
  GeometryId id_B = GeometryId::get_new_id();
  GeometryIndex index_A(0);
  GeometryIndex index_B(1);
  ProximityIndex engine_index_A =
      engine.AddDynamicGeometry(Box(0.49, 0.63, 0.015), index_A);
  ProximityIndex engine_index_B =
      engine.AddDynamicGeometry(Cylinder(0.08, 0.002), index_B);
  ASSERT_EQ(engine_index_A, 0);
  ASSERT_EQ(engine_index_B, 1);
  std::vector<GeometryIndex> indices{index_A, index_B};
  Isometry3<double> X_WA;
  // Original translation was p_WA = (-0.145, -0.63, 0.2425) and
  // p_WB = (0, -0.6, 0.251), respectively.
  // clang-format off
  X_WA.matrix() << 0, -1, 0, -0.25,
                   1,  0, 0,     0,
                   0,  0, 1,     0,
                   0,  0, 0,     1;
  Isometry3<double> X_WB;
  X_WB.matrix() << 1, 0, 0,      0,
                   0, 1, 0,      0,
                   0, 0, 1, 0.0085,
                   0, 0, 0,      1;
  // clang-format on
  std::vector<Isometry3<double>> X_WG{X_WA, X_WB};
  engine.UpdateWorldPoses(X_WG, indices);
  std::vector<GeometryId> geometry_map{id_A, id_B};
  auto pairs = engine.ComputePointPairPenetration(geometry_map);
  EXPECT_EQ(pairs.size(), 0);
}

// When anchored geometry is added to the proximity engine, the broadphase
// algorithm needs to be properly updated, otherwise it assumes all of the
// anchored geometry has the identity transformation. This test confirms that
// this configuration occurs; the dynamic sphere and anchored sphere are
// configured away from the origin in collision. Without proper broadphase
// initialization for the anchored geometry, no collision is reported.
GTEST_TEST(ProximityEngineTests, AnchoredBroadPhaseInitialization) {
  ProximityEngine<double> engine;
  GeometryId id_D = GeometryId::get_new_id();
  GeometryId id_A = GeometryId::get_new_id();
  GeometryIndex index_D(0);
  GeometryIndex index_A(1);
  ProximityIndex engine_index_D =
      engine.AddDynamicGeometry(Sphere(0.5), index_D);

  Isometry3<double> X_WA{Translation3d{-3, 0, 0}};
  ProximityIndex engine_index_A =
      engine.AddAnchoredGeometry(Sphere(0.5), X_WA, index_A);

  // These should have the same index value because one draws from the dynamic
  // set, the other from the anchored.
  ASSERT_EQ(engine_index_D, 0);
  ASSERT_EQ(engine_index_A, 0);

  Isometry3<double> X_WD{Translation3d{-3, 0.75, 0}};
  // NOTE: The vector of indicies should *only* be the indices of the dynamic
  // geometry. The only requirement for the vector of poses is that the
  // poses[indices[i]] must be defined and should map to the geometry with
  // index indices[i]. In this case, a single dynamic geometry with index 0,
  // means it is sufficient to create a pose vector with a single pose.
  engine.UpdateWorldPoses({X_WD}, {index_D});
  std::vector<GeometryId> geometry_map{id_D, id_A};
  auto pairs = engine.ComputePointPairPenetration(geometry_map);
  EXPECT_EQ(pairs.size(), 1);

  // Confirm that it survives copying.
  ProximityEngine<double> engine_copy(engine);
  engine_copy.UpdateWorldPoses({X_WD}, {index_D});
  auto pairs_copy = engine_copy.ComputePointPairPenetration(geometry_map);
  EXPECT_EQ(pairs_copy.size(), 1);
}

// Basic smoke test for the autodiffibility of the signed distance computation.
// Tests against the anchored geometry. Specifically, it confirms that while
// poses are set with double, the calculation is done with AutoDiff and
// derivatives come through.
GTEST_TEST(ProximityEngineTests, ComputeSignedDistanceAutoDiffAnchored) {
  ProximityEngine<AutoDiffXd> engine;

  const double kEps = std::numeric_limits<double>::epsilon();

  // Given a sphere with radius 0.7, centered on p_WSo, the point p_SQ = (2,3,6)
  // is outside G at the positive distance 6.3.
  const double expected_distance = 6.3;
  const Vector3d p_SQ_W{2.0, 3.0, 6.0};
  const Vector3d p_WS_W{0.5, 1.25, -2};
  const Vector3d p_WQ{p_SQ_W + p_WS_W};
  Vector3<AutoDiffXd> p_WQ_ad = math::initializeAutoDiff(p_WQ);

  // An empty world inherently produces no results.
  {
    std::vector<GeometryId> geometry_map;
    std::vector<Isometry3<AutoDiffXd>> X_WGs;
    const auto results = engine.ComputeSignedDistanceToPoint(
        p_WQ_ad, geometry_map, X_WGs);
    EXPECT_EQ(results.size(), 0);
  }

  // Against an anchored sphere.
  {
    const RigidTransformd X_WS = RigidTransformd(p_WS_W);
    engine.AddAnchoredGeometry(Sphere(0.7), X_WS, GeometryIndex(0));
    std::vector<GeometryId> geometry_map{GeometryId::get_new_id()};
    const RigidTransform<AutoDiffXd> X_WS_ad = X_WS.cast<AutoDiffXd>();
    std::vector<Isometry3<AutoDiffXd>> X_WGs{X_WS_ad};

    // Distance is just beyond the threshold.
    {
      std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
          engine.ComputeSignedDistanceToPoint(p_WQ_ad, geometry_map, X_WGs,
                                              expected_distance - 1e-14);
      EXPECT_EQ(results.size(), 0);
    }

    // Distance is just within the threshold
    {
      std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
          engine.ComputeSignedDistanceToPoint(p_WQ_ad, geometry_map, X_WGs,
                                              expected_distance + 1e-14);
      EXPECT_EQ(results.size(), 1);
      const SignedDistanceToPoint<AutoDiffXd>& distance_data = results[0];
      // The autodiff seems to lose a couple of bits relative to the known
      // answer.
      EXPECT_NEAR(distance_data.distance.value(), expected_distance, 4 * kEps);
      // The analytical `grad_W` value should match the autodiff-computed
      // gradient.
      const Vector3d ddistance_dp_WQ = distance_data.distance.derivatives();
      const Vector3d grad_W = math::autoDiffToValueMatrix(distance_data.grad_W);
      EXPECT_TRUE(CompareMatrices(ddistance_dp_WQ, grad_W, kEps));
    }
  }
}

// Basic smoke test for the autodiffibility of the signed distance computation.
// Tests against the dynamic geometry. Specifically, it confirms that while
// poses are set with double, the calculation is done with AutoDiff and
// derivatives come through.
GTEST_TEST(ProximityEngineTests, ComputeSignedDistanceAutoDiffDynamic) {
  ProximityEngine<AutoDiffXd> engine;

  const double kEps = std::numeric_limits<double>::epsilon();

  // Given a sphere with radius 0.7, centered on p_WSo, the point p_SQ = (2,3,6)
  // is outside G at the positive distance 6.3.
  const double expected_distance = 6.3;
  const Vector3d p_SQ{2.0, 3.0, 6.0};
  const Vector3d p_WS{0.5, 1.25, -2};
  const Vector3d p_WQ{p_SQ + p_WS};
  Vector3<AutoDiffXd> p_WQ_ad = math::initializeAutoDiff(p_WQ);

  // Against a dynamic sphere.
  GeometryIndex index(0);
  engine.AddDynamicGeometry(Sphere(0.7), index);
  std::vector<GeometryId> geometry_map{GeometryId::get_new_id()};
  const auto X_WS_ad =
      RigidTransformd(p_WS).cast<AutoDiffXd>().GetAsIsometry3();
  std::vector<Isometry3<AutoDiffXd>> X_WGs{X_WS_ad};
  engine.UpdateWorldPoses(X_WGs, {index});

  // Distance is just beyond the threshold.
  {
    std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
        engine.ComputeSignedDistanceToPoint(p_WQ_ad, geometry_map, X_WGs,
                                            expected_distance - 1e-14);
    EXPECT_EQ(results.size(), 0);
  }

  // Distance is just within the threshold
  {
    std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
        engine.ComputeSignedDistanceToPoint(p_WQ_ad, geometry_map, X_WGs,
                                            expected_distance + 1e-14);
    EXPECT_EQ(results.size(), 1);
    const SignedDistanceToPoint<AutoDiffXd>& distance_data = results[0];
    // The autodiff seems to lose a couple of bits relative to the known
    // answer.
    EXPECT_NEAR(distance_data.distance.value(), expected_distance, 4 * kEps);
    // The analytical `grad_W` value should match the autodiff-computed
    // gradient.
    const Vector3d ddistance_dp_WQ = distance_data.distance.derivatives();
    const Vector3d grad_W = math::autoDiffToValueMatrix(distance_data.grad_W);
    EXPECT_TRUE(CompareMatrices(ddistance_dp_WQ, grad_W, kEps));
  }
}


}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
