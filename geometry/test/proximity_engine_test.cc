#include "drake/geometry/proximity_engine.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {

class ProximityEngineTester {
 public:
  ProximityEngineTester() = delete;

  template <typename T>
  static bool IsDeepCopy(const ProximityEngine<T>& test_engine,
                         const ProximityEngine<T>& ref_engine) {
    return ref_engine.IsDeepCopy(test_engine);
  }
};

namespace {

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Translation3d;
using std::move;

// Test simple addition of dynamic geometry.
GTEST_TEST(ProximityEngineTests, AddDynamicGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  GeometryIndex index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(index, 0);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests simple addition of anchored geometry.
GTEST_TEST(ProximityEngineTests, AddAchoredGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex index = engine.AddAnchoredGeometry(sphere, pose);
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
  AnchoredGeometryIndex a_index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(g_index, 0);
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests the copy semantics of the ProximityEngine -- the copy is a complete,
// deep copy.
GTEST_TEST(ProximityEngineTests, CopySemantics) {
  ProximityEngine<double> ref_engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex a_index = ref_engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = ref_engine.AddDynamicGeometry(sphere);
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
  AnchoredGeometryIndex a_index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = engine.AddDynamicGeometry(sphere);
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


// Penetration tests

// A scene with no geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationOnEmptyScene) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> empty_map;

  auto results = engine.ComputePointPairPenetration(empty_map, empty_map);
  EXPECT_EQ(results.size(), 0);
}

// A scene with a single anchored geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationSingleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> dynamic_map;
  std::vector<GeometryId> anchored_map;

  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex index = engine.AddAnchoredGeometry(sphere, pose);
  anchored_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index, 0);
  auto results = engine.ComputePointPairPenetration(dynamic_map, anchored_map);
  EXPECT_EQ(results.size(), 0);
}

// Tests that anchored geometry aren't collided against each other -- even if
// they actually *are* in penetration.
GTEST_TEST(ProximityEngineTests, PenetrationMultipleAnchored) {
  ProximityEngine<double> engine;
  std::vector<GeometryId> dynamic_map;
  std::vector<GeometryId> anchored_map;

  const double radius = 0.5;
  Sphere sphere{radius};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex index1 = engine.AddAnchoredGeometry(sphere, pose);
  anchored_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index1, 0);
  pose.translation() << 1.8 * radius, 0, 0;
  AnchoredGeometryIndex index2 = engine.AddAnchoredGeometry(sphere, pose);
  anchored_map.push_back(GeometryId::get_new_id());
  EXPECT_EQ(index2, 1);
  auto results = engine.ComputePointPairPenetration(dynamic_map, anchored_map);
  EXPECT_EQ(results.size(), 0);
}

// These tests validate collisions between spheres. This does *not* test against
// other geometry types because we assume FCL works. This merely confirms that
// the ProximityEngine functions provide the correct mapping.

// Common class for evaluating a simple penetration case between two spheres.
// The variations are in sphere *ownership* (see below).
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
    engine->UpdateWorldPoses(poses);
  }

  // Compute penetration and confirm that a single penetration with the expected
  // properties was found. Provide the engine indices of the sphere located at
  // the origin and the sphere positioned to be in collision.
  template <typename T>
  void ExpectPenetration(GeometryId origin_sphere, GeometryId colliding_sphere,
                         ProximityEngine<T>* engine) {
    std::vector<PenetrationAsPointPair<double>> results =
        engine->ComputePointPairPenetration(dynamic_map_, anchored_map_);
    EXPECT_EQ(results.size(), 1);
    const PenetrationAsPointPair<double> penetration = results[0];

    // There are no guarantees as to the ordering of which element is A and
    // which is B. This test enforces an order for validation.

    // First confirm membership
    EXPECT_TRUE((penetration.id_A == origin_sphere &&
                 penetration.id_B == colliding_sphere) ||
                (penetration.id_A == colliding_sphere &&
                 penetration.id_B == origin_sphere));

    // Assume A => origin_sphere and b => colliding_sphere
    // NOTE: In this current version, penetration is only reported in double.
    PenetrationAsPointPair<double> expected;
    expected.id_A = origin_sphere;  // located at origin
    expected.id_B = colliding_sphere;  // located at [1.5R, 0, 0]
    expected.depth = 2 * radius_ - colliding_x_;
    expected.p_WCa = Vector3<double>{radius_, 0, 0};
    expected.p_WCb = Vector3<double>{colliding_x_ - radius_, 0, 0};
    expected.nhat_BA_W = -Vector3<double>::UnitX();

    // Reverse if previous order assumption is false
    if (penetration.id_A == colliding_sphere) {
      Vector3<double> temp;
      // Swap the indices
      expected.id_A = colliding_sphere;
      expected.id_B = origin_sphere;
      // Swap the points
      temp = expected.p_WCa;
      expected.p_WCa = expected.p_WCb;
      expected.p_WCb = temp;
      // Reverse the normal
      expected.nhat_BA_W = -expected.nhat_BA_W;
      // Penetration depth is same either way; do nothing.
    }

    EXPECT_EQ(penetration.id_A, expected.id_A);
    EXPECT_EQ(penetration.id_B, expected.id_B);
    EXPECT_EQ(penetration.depth, expected.depth);
    EXPECT_TRUE(CompareMatrices(penetration.p_WCa, expected.p_WCa, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.p_WCb, expected.p_WCb, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.nhat_BA_W, expected.nhat_BA_W,
                                1e-13, MatrixCompareType::absolute));
  }

  // Compute penetration and confirm that none were found.
  void ExpectNoPenetration(ProximityEngine<double>* engine = nullptr) {
    engine = engine == nullptr ? &engine_ : engine;
    std::vector<PenetrationAsPointPair<double>> results =
        engine->ComputePointPairPenetration(dynamic_map_, anchored_map_);
    EXPECT_EQ(results.size(), 0);
  }

  ProximityEngine<double> engine_;
  std::vector<GeometryId> dynamic_map_;
  std::vector<GeometryId> anchored_map_;
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
  AnchoredGeometryIndex anchored_index =
      engine_.AddAnchoredGeometry(sphere_, pose);
  GeometryId origin_id = GeometryId::get_new_id();
  anchored_map_.push_back(origin_id);
  EXPECT_EQ(anchored_index, 0);

  // Set up dynamic geometry
  GeometryIndex dynamic_index = engine_.AddDynamicGeometry(sphere_);
  GeometryId dynamic_id = GeometryId::get_new_id();
  dynamic_map_.push_back(dynamic_id);
  EXPECT_EQ(dynamic_index, 0);
  EXPECT_EQ(engine_.num_geometries(), 2);

  // Non-colliding case
  MoveDynamicSphere(dynamic_index, false /* not colliding */);
  ExpectNoPenetration();

  // Colliding case
  MoveDynamicSphere(dynamic_index, true /* colliding */);
  ExpectPenetration(origin_id, dynamic_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(origin_id, dynamic_id, &copy_engine);

  // Test AutoDiff converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine = engine_.ToAutoDiff();
  ExpectPenetration(origin_id, dynamic_id, ad_engine.get());
}

// Performs the same collision test between two dynamic spheres which belong to
// the same source
TEST_F(SimplePenetrationTest, PenetrationDynamicAndDynamicSingleSource) {
  GeometryIndex origin_index = engine_.AddDynamicGeometry(sphere_);
  GeometryId origin_id = GeometryId::get_new_id();
  dynamic_map_.push_back(origin_id);
  EXPECT_EQ(origin_index, 0);
  std::vector<Isometry3<double>> poses{Isometry3<double>::Identity()};
  engine_.UpdateWorldPoses(poses);

  GeometryIndex collide_index = engine_.AddDynamicGeometry(sphere_);
  GeometryId collide_id = GeometryId::get_new_id();
  dynamic_map_.push_back(collide_id);
  EXPECT_EQ(collide_index, 1);
  EXPECT_EQ(engine_.num_geometries(), 2);

  // Non-colliding case
  MoveDynamicSphere(collide_index, false /* not colliding */);
  ExpectNoPenetration();

  // Colliding case
  MoveDynamicSphere(collide_index, true /* colliding */);
  ExpectPenetration(origin_id, collide_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(origin_id, collide_id, &copy_engine);

  // Test AutoDiff converted engine
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine = engine_.ToAutoDiff();
  ExpectPenetration(origin_id, collide_id, ad_engine.get());
}

class CylinderVsHalfPlantPenetrationTest : public ::testing::Test {
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
  void MoveDynamicSphere(int index, bool is_colliding) {
    std::vector<Isometry3<double>> poses(engine_.num_dynamic(),
                                         Isometry3<double>::Identity());
    const double z_pos = is_colliding ? colliding_z_ : free_z_;
    poses[index] = Isometry3<double>(Translation3d{0, 0, z_pos});
    engine_.UpdateWorldPoses(poses);
  }

  // Compute penetration and confirm that a single penetration with the expected
  // properties was found. Provide the engine indices of the sphere located at
  // the origin and the sphere positioned to be in collision.
  void ExpectPenetration(GeometryId half_space, GeometryId cylinder) {
    std::vector<PenetrationAsPointPair<double>> results =
        engine_.ComputePointPairPenetration(dynamic_map_, anchored_map_);
    EXPECT_EQ(results.size(), 1);
    const PenetrationAsPointPair<double> penetration = results[0];

    // There are no guarantees as to the ordering of which element is A and
    // which is B. This test enforces an order for validation.

    // First confirm membership
    EXPECT_TRUE((penetration.id_A == half_space &&
        penetration.id_B == cylinder) ||
        (penetration.id_A == cylinder &&
            penetration.id_B == half_space));

    // Assume A => half_space and b => cylinder
    // NOTE: In this current version, penetration is only reported in double.
    PenetrationAsPointPair<double> expected;
    expected.id_A = half_space;
    expected.id_B = cylinder;
    expected.depth = length_/2.0 - colliding_z_;
    expected.p_WCa = Vector3<double>{0, 0, 0};
    expected.p_WCb = Vector3<double>{0, 0, colliding_z_ - length_/2};
    expected.nhat_BA_W = -Vector3<double>::UnitZ();

    // Reverse if previous order assumption is false
    if (penetration.id_A == cylinder) {
      Vector3<double> temp;
      // Swap the indices
      expected.id_A = cylinder;
      expected.id_B = half_space;
      // Swap the points
      temp = expected.p_WCa;
      expected.p_WCa = expected.p_WCb;
      expected.p_WCb = temp;
      // Reverse the normal
      expected.nhat_BA_W = -expected.nhat_BA_W;
      // Penetration depth is same either way; do nothing.
    }

    EXPECT_EQ(penetration.id_A, expected.id_A);
    EXPECT_EQ(penetration.id_B, expected.id_B);
    EXPECT_EQ(penetration.depth, expected.depth);
    EXPECT_TRUE(CompareMatrices(penetration.p_WCa, expected.p_WCa, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.p_WCb, expected.p_WCb, 1e-13,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(penetration.nhat_BA_W, expected.nhat_BA_W,
                                1e-13, MatrixCompareType::absolute));
  }

  // Compute penetration and confirm that none were found.
  void ExpectNoPenetration(ProximityEngine<double>* engine = nullptr) {
    std::vector<PenetrationAsPointPair<double>> results =
        engine_.ComputePointPairPenetration(dynamic_map_, anchored_map_);
    EXPECT_EQ(results.size(), 0);
  }
  
  ProximityEngine<double> engine_;
  std::vector<GeometryId> dynamic_map_;   // Ordered by GeometryIndex
  std::vector<GeometryId> anchored_map_;  // Ordered by AnchoredGeometryIndex
  const double radius_{0.05};
  const double length_{4 * radius_};
  const Cylinder cylinder_{radius_, length_};
  const HalfSpace half_space_;
  const double free_z_{0.3};
  const double colliding_z_{0.049};
};

TEST_F(CylinderVsHalfPlantPenetrationTest, PenetrationTest) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  // Set up anchored geometry
  Isometry3<double> pose =
      HalfSpace::MakePose(Vector3d::UnitZ() /*normal*/,
                          Vector3d::Zero() /*point*/);
  AnchoredGeometryIndex anchored_index =
      engine_.AddAnchoredGeometry(half_space_, pose);
  GeometryId half_space_geometry_id = GeometryId::get_new_id();
  anchored_map_.push_back(half_space_geometry_id);
  EXPECT_EQ(anchored_index, 0);

  // Set up dynamic geometry
  GeometryIndex dynamic_index = engine_.AddDynamicGeometry(cylinder_);
  GeometryId cylinder_geometry_id = GeometryId::get_new_id();
  dynamic_map_.push_back(cylinder_geometry_id);
  EXPECT_EQ(dynamic_index, 0);
  EXPECT_EQ(engine_.num_geometries(), 2);

  std::vector<Isometry3<double>> poses(
      engine_.num_dynamic(), Isometry3<double>::Identity());
  std::vector<PenetrationAsPointPair<double>> results;

  // Non-colliding case
  poses[dynamic_index] = Isometry3<double>(Translation3d{0, 0, free_z_});
  engine_.UpdateWorldPoses(poses);
  results = engine_.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 0);

  // Colliding case
  poses[dynamic_index] = Isometry3<double>(Translation3d{0, 0, colliding_z_});
  engine_.UpdateWorldPoses(poses);
  results = engine_.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  EXPECT_NEAR(results[0].depth, 0.051, kTolerance);
  PRINT_VAR(results[0].depth);

  // Rotate cylinder 180 degrees about its center.
  // Exactly as the previous case but cylinder is rotated 180 degrees, results
  // should not change.
  poses[dynamic_index].linear() = AngleAxisd(M_PI, Vector3d::UnitX()).matrix();
  poses[dynamic_index].translation() = Vector3d(0, 0, colliding_z_);
  engine_.UpdateWorldPoses(poses);
  results = engine_.ComputePointPairPenetration(dynamic_map_, anchored_map_);
  EXPECT_EQ(results.size(), 1);
  // We expect the same result since the cylinder was only rotated 180 degrees
  // about (presumably) its center.
  EXPECT_NEAR(results[0].depth, 0.051, kTolerance);
  PRINT_VAR(results[0].depth);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
