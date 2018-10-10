#include "drake/geometry/dev/proximity_engine.h"

#include <utility>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace dev {
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
};

namespace {

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using std::move;

// Tests for manipulating the population of the proximity engine.

// Test simple addition of dynamic geometry.
GTEST_TEST(ProximityEngineTests, AddDynamicGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  ProximityIndex index = engine.AddDynamicGeometry(sphere, InternalIndex(0));
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
                                                    InternalIndex(0));
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
                                                      InternalIndex(0));
  EXPECT_EQ(a_index, 0);
  ProximityIndex g_index = engine.AddDynamicGeometry(sphere, InternalIndex(0));
  EXPECT_EQ(g_index, 0);
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests for reading .obj files.------------------------------------------------

// Tests exception when we read an .obj file with two objects into Convex
GTEST_TEST(ProximityEngineTests, ExceptionTwoObjectsInObjFileForConvex) {
  ProximityEngine<double> engine;
  Convex convex{drake::FindResourceOrThrow(
      "drake/geometry/dev/test/forbidden_two_cubes.obj"), 1.0};
  DRAKE_EXPECT_THROWS_MESSAGE(engine.AddDynamicGeometry(convex,
                                                        InternalIndex(0)),
      std::runtime_error, ".*one and only one object.*");
}

// Tests for copy/move semantics.  ---------------------------------------------

// Tests the copy semantics of the ProximityEngine -- the copy is a complete,
// deep copy.
GTEST_TEST(ProximityEngineTests, CopySemantics) {
  ProximityEngine<double> ref_engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  ProximityIndex a_index =
      ref_engine.AddAnchoredGeometry(sphere, pose, InternalIndex(0));
  EXPECT_EQ(a_index, 0);
  ProximityIndex g_index =
      ref_engine.AddDynamicGeometry(sphere, InternalIndex(0));
  EXPECT_EQ(g_index, 0);

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
                                                      InternalIndex(0));
  EXPECT_EQ(a_index, 0);
  ProximityIndex g_index = engine.AddDynamicGeometry(sphere,
                                                     InternalIndex(0));
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
                                                    InternalIndex(0));
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
                                                     InternalIndex(0));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index1, 0);
  pose.translation() << 1.8 * radius, 0, 0;
  ProximityIndex index2 = engine.AddAnchoredGeometry(sphere, pose,
                                                     InternalIndex(1));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index2, 1);
  const auto results = engine.ComputeSignedDistancePairwiseClosestPoints(
      geometry_map);
  EXPECT_EQ(results.size(), 0);
}
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
                                                    InternalIndex(0));
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
                                                     InternalIndex(0));
  geometry_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index1, 0);
  pose.translation() << 1.8 * radius, 0, 0;
  ProximityIndex index2 = engine.AddAnchoredGeometry(sphere, pose,
                                                     InternalIndex(1));
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
    std::vector<InternalIndex> indices(poses.size());
    std::iota(indices.begin(), indices.end(), InternalIndex(0));
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
    // TODO(hongkai.dai): Set the FCL solver tolerance, and check the distance
    // against that tolerance, when the PR
    // https://github.com/flexible-collision-library/fcl/pull/314 is merged into
    // FCL upstream.
    CompareSignedDistancePair(distance, expected_distance, 2e-3);
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

    CompareSignedDistancePair(distance, expected_distance, 2e-3);
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
      engine_.AddAnchoredGeometry(sphere_, pose, InternalIndex(0));
  EXPECT_EQ(anchored_index, 0);
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);

  // Set up dynamic geometry
  ProximityIndex dynamic_index =
      engine_.AddDynamicGeometry(sphere_, InternalIndex(1));
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
      engine_.AddDynamicGeometry(sphere_, InternalIndex(0));
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);
  EXPECT_EQ(origin_index, 0);
  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  std::vector<InternalIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), InternalIndex(0));
  engine_.UpdateWorldPoses(poses, indices);

  ProximityIndex collide_index =
      engine_.AddDynamicGeometry(sphere_, InternalIndex(1));
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
  InternalIndex dynamic1(0);
  engine_.AddDynamicGeometry(sphere_, dynamic1);
  InternalIndex dynamic2(1);
  engine_.AddDynamicGeometry(sphere_, dynamic2);

  Isometry3<double> pose{Isometry3<double>::Identity()};
  InternalIndex anchored1(0);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored1);
  InternalIndex anchored2(1);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored2);

  int expected_clique = PET::peek_next_clique(engine_);

  // No dynamic geometry --> no cliques generated.
  engine_.ExcludeCollisionsWithin({}, {anchored1, anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // Single dynamic and no anchored geometry --> no cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1}, {});
  ASSERT_EQ(PET::peek_next_clique(engine_), expected_clique);

  // Multiple dynamic and no anchored geometry --> cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1, InternalIndex(1)}, {});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);

  // Single dynamic and (one or more) anchored geometry --> cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1}, {anchored1});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
  engine_.ExcludeCollisionsWithin({dynamic1}, {anchored1, anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);

  // Multiple dynamic and (one or more) anchored geometry --> cliques generated.
  engine_.ExcludeCollisionsWithin({dynamic1, dynamic2}, {anchored1});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
  engine_.ExcludeCollisionsWithin({dynamic1, dynamic2}, {anchored1, anchored2});
  ASSERT_EQ(PET::peek_next_clique(engine_), ++expected_clique);
}

// Performs the same collision test where the geometries have been filtered.
TEST_F(SimplePenetrationTest, ExcludeCollisionsWithin) {
  InternalIndex origin_index(0);
  engine_.AddDynamicGeometry(sphere_, origin_index);
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);

  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  std::vector<InternalIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), InternalIndex(0));
  engine_.UpdateWorldPoses(poses, indices);

  InternalIndex collide_index(1);
  engine_.AddDynamicGeometry(sphere_, collide_index);
  GeometryId collide_id = GeometryId::get_new_id();
  geometry_map_.push_back(collide_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  engine_.ExcludeCollisionsWithin({origin_index, collide_index}, {});

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
  InternalIndex dynamic1(0);
  engine_.AddDynamicGeometry(sphere_, dynamic1);
  InternalIndex dynamic2(1);
  engine_.AddDynamicGeometry(sphere_, dynamic2);
  InternalIndex dynamic3(2);
  engine_.AddDynamicGeometry(sphere_, dynamic3);

  Isometry3<double> pose{Isometry3<double>::Identity()};
  InternalIndex anchored1(0);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored1);
  InternalIndex anchored2(1);
  engine_.AddAnchoredGeometry(sphere_, pose, anchored2);
  InternalIndex anchored3(2);
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
  InternalIndex origin_index(0);
  engine_.AddDynamicGeometry(sphere_, origin_index);
  GeometryId origin_id = GeometryId::get_new_id();
  geometry_map_.push_back(origin_id);

  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  std::vector<InternalIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), InternalIndex(0));
  engine_.UpdateWorldPoses(poses, indices);

  InternalIndex collide_index(1);
  engine_.AddDynamicGeometry(sphere_, collide_index);
  GeometryId collide_id = GeometryId::get_new_id();
  geometry_map_.push_back(collide_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  engine_.ExcludeCollisionsBetween({origin_index}, {}, {collide_index}, {});

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
  InternalIndex box_index(0);
  engine.AddDynamicGeometry(Box{w, h, d}, box_index);
  InternalIndex sphere_index(1);
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
  std::vector<InternalIndex> indices(poses.size());
  std::iota(indices.begin(), indices.end(), InternalIndex(0));
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
      Isometry3d X_WB = func(p_BoC_B_, p_WC_);
      // Confirm that p_BoC_B transforms to p_WC.
      Vector3d p_WC_test = X_WB * p_BoC_B_;
      EXPECT_TRUE(CompareMatrices(p_WC_test, p_WC_, 1e-15,
                                  MatrixCompareType::absolute));

      // Confirm that *all* other points map to values where z > 0.
      for (double x : {-0.5, 0.5}) {
        for (double y : {-0.5, 0.5}) {
          for (double z : {-0.5, 0.5}) {
            Vector3d p_BoC_B{x, y, z};
            if (p_BoC_B.isApprox(p_BoC_B_)) {
              continue;
            }
            Vector3d p_WC = X_WB * p_BoC_B;
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
                     const Isometry3d& X_WB) {
    InternalIndex tangent_index(0);
    engine_.AddDynamicGeometry(shape(shape_type), tangent_index);
    GeometryId tangent_id = GeometryId::get_new_id();
    geometry_map_.push_back(tangent_id);

    InternalIndex box_index(1);
    engine_.AddDynamicGeometry(box_, box_index);
    GeometryId box_id = GeometryId::get_new_id();
    geometry_map_.push_back(box_id);

    // Confirm that there are no other geometries interfering.
    EXPECT_EQ(tangent_index, 0);
    EXPECT_EQ(box_index, 1);
    ASSERT_EQ(engine_.num_dynamic(), 2);

    // Update the poses of the geometry.
    std::vector<Isometry3d> poses{shape_pose(shape_type), X_WB};
    std::vector<InternalIndex> indices(poses.size());
    std::iota(indices.begin(), indices.end(), InternalIndex(0));
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
  static Isometry3d X_WB_2(const Vector3d& p_BoC_B, const Vector3d& p_WC) {
    // Compute the pose of the colliding box.
    // a. Orient the box so that the corner p_BoC_B = (-0.5, -0.5, -0.5) lies in
    //    the most -z extent. With only rotation, p_BoC_B != p_WC.
    Isometry3d X_WB;
    X_WB = AngleAxisd(std::atan(M_SQRT2), Vector3d(M_SQRT1_2, -M_SQRT1_2, 0))
               .toRotationMatrix();

    // b. Translate it so that the rotated corner p_BoC_W lies at
    //    (0, 0, -d).
    Vector3d p_BoC_W = X_WB.linear() * p_BoC_B;
    Vector3d p_WB = p_WC - p_BoC_W;
    X_WB.translation() = p_WB;
    return X_WB;
  }

  // Produces the X_WB that produces low-quality answers.
  static Isometry3d X_WB_1(const Vector3d& p_BoC_B, const Vector3d& p_WC) {
    // Compute the pose of the colliding box.
    // a. Orient the box so that the corner p_BoC_B = (-0.5, -0.5, -0.5) lies in
    //    the most -z extent. With only rotation, p_BoC_B != p_WC.
    Isometry3d X_WB;
    X_WB = AngleAxisd(-M_PI_4, Vector3d::UnitY()) *
           AngleAxisd(M_PI_4, Vector3d::UnitX());

    // b. Translate it so that the rotated corner p_BoC_W lies at
    //    (0, 0, -d).
    Vector3d p_BoC_W = X_WB.linear() * p_BoC_B;
    Vector3d p_WB = p_WC - p_BoC_W;
    X_WB.translation() = p_WB;
    return X_WB;
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
      "drake/geometry/dev/test/quad_cube.obj"), 5.0};

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

}  // namespace
}  // namespace internal
}  // namespace dev
}  // namespace geometry
}  // namespace drake
