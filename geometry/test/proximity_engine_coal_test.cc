#include <limits>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/collision_backend.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/math/rigid_transform.h"

/* @file A smoke test for the Coal backend.

 This test uses the explicit-backend constructor to confirm that a Coal-backed
 engine registers geometry and answers each family of query at all.

 It deliberately does *not* characterize Coal's accuracy. The tolerances are
 loose on purpose: they ask "did the query run and produce a sane answer", not
 "how good is the answer".  */

namespace drake {
namespace geometry {
namespace internal {

/* ProximityEngine's introspection accessors are private and befriend this
 name; see proximity_engine_test.cc, which does the same. */
class ProximityEngineTester {
 public:
  ProximityEngineTester() = delete;

  template <typename T>
  static CollisionBackend backend(const ProximityEngine<T>& engine) {
    return engine.collision_backend();
  }

  template <typename T>
  static bool IsDeepCopy(const ProximityEngine<T>& test_engine,
                         const ProximityEngine<T>& ref_engine) {
    return ref_engine.IsDeepCopy(test_engine);
  }
};

namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using std::unordered_map;
using std::vector;

constexpr double kInf = std::numeric_limits<double>::infinity();

/* Two unit spheres whose centers are 1.5 apart, so they overlap by 0.5. */
class CoalEngineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ProximityProperties props;
    engine_.AddDynamicGeometry(Sphere(1.0), RigidTransformd{}, id_A_, props);
    engine_.AddDynamicGeometry(Sphere(1.0), RigidTransformd{}, id_B_, props);
    X_WGs_ = {{id_A_, RigidTransformd{}},
              {id_B_, RigidTransformd{Vector3d{1.5, 0, 0}}}};
    engine_.UpdateWorldPoses(X_WGs_);
  }

  ProximityEngine<double> engine_{CollisionBackend::kCoal};
  const GeometryId id_A_{GeometryId::get_new_id()};
  const GeometryId id_B_{GeometryId::get_new_id()};
  unordered_map<GeometryId, RigidTransformd> X_WGs_;
};

TEST_F(CoalEngineTest, ReportsItsBackend) {
  EXPECT_EQ(ProximityEngineTester::backend(engine_), CollisionBackend::kCoal);
  // The default-constructed engine follows the selector, which in turn
  // follows DRAKE_PROXIMITY_BACKEND; assert against the selector rather than
  // against a hard-coded FCL, so this test is correct under either setting.
  const ProximityEngine<double> default_engine;
  EXPECT_EQ(ProximityEngineTester::backend(default_engine),
            GetDefaultCollisionBackend());
}

TEST_F(CoalEngineTest, RegistersGeometry) {
  EXPECT_EQ(engine_.num_geometries(), 2);
  EXPECT_EQ(engine_.num_dynamic(), 2);
}

TEST_F(CoalEngineTest, PointPairPenetration) {
  const vector<PenetrationAsPointPair<double>> pairs =
      engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(pairs.size(), 1);
  // The spheres overlap by 2 - 1.5 = 0.5. The depth must come out positive;
  // Coal reports a signed distance here and the policy negates it (D3), so a
  // sign error would show up as a missing or negative result.
  EXPECT_NEAR(pairs[0].depth, 0.5, 1e-9);
}

TEST_F(CoalEngineTest, SignedDistancePairwise) {
  const vector<SignedDistancePair<double>> pairs =
      engine_.ComputeSignedDistancePairwiseClosestPoints(X_WGs_, kInf);
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_NEAR(pairs[0].distance, -0.5, 1e-9);
}

TEST_F(CoalEngineTest, SignedDistanceToPoint) {
  // A point 3 units along +x is 2 units from sphere A's surface and 0.5 from
  // sphere B's.
  const vector<SignedDistanceToPoint<double>> distances =
      engine_.ComputeSignedDistanceToPoint(Vector3d{3, 0, 0}, X_WGs_, kInf);
  ASSERT_EQ(distances.size(), 2);
  for (const auto& d : distances) {
    const double expected = (d.id_G == id_A_) ? 2.0 : 0.5;
    EXPECT_NEAR(d.distance, expected, 1e-9);
  }
}

TEST_F(CoalEngineTest, CollisionCandidatesAndHasCollisions) {
  EXPECT_TRUE(engine_.HasCollisions());
  const vector<SortedPair<GeometryId>> candidates =
      engine_.FindCollisionCandidates();
  ASSERT_EQ(candidates.size(), 1);
  EXPECT_EQ(candidates[0], SortedPair<GeometryId>(id_A_, id_B_));

  // Move them apart; the queries must agree that they no longer touch.
  X_WGs_.at(id_B_) = RigidTransformd{Vector3d{5, 0, 0}};
  engine_.UpdateWorldPoses(X_WGs_);
  EXPECT_FALSE(engine_.HasCollisions());
  EXPECT_EQ(engine_.ComputePointPairPenetration(X_WGs_).size(), 0);
}

/* A convex hull exercises D4: Drake's polygon face encoding has to be
 fan-triangulated for Coal, and a mistake there would mis-shape the geometry
 rather than fail to compile. A box convex hull of half-extent 1, centered 1.5
 from the unit sphere, overlaps it by 0.5 just as the sphere pair did. */
TEST_F(CoalEngineTest, ConvexHullGeometry) {
  ProximityEngine<double> engine{CollisionBackend::kCoal};
  const GeometryId id_sphere = GeometryId::get_new_id();
  const GeometryId id_convex = GeometryId::get_new_id();
  ProximityProperties props;
  engine.AddDynamicGeometry(Sphere(1.0), RigidTransformd{}, id_sphere, props);
  engine.AddDynamicGeometry(Box(2.0, 2.0, 2.0), RigidTransformd{}, id_convex,
                            props);

  const unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_sphere, RigidTransformd{}},
      {id_convex, RigidTransformd{Vector3d{2.5, 0, 0}}}};
  engine.UpdateWorldPoses(X_WGs);

  const vector<SignedDistancePair<double>> pairs =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
  ASSERT_EQ(pairs.size(), 1);
  // Sphere surface at x = 1, box face at x = 1.5.
  EXPECT_NEAR(pairs[0].distance, 0.5, 1e-9);
}

/* Copying and scalar conversion must stay on the backend they started on. */
TEST_F(CoalEngineTest, CopyAndScalarConversionPreserveTheBackend) {
  const ProximityEngine<double> copied_engine(engine_);
  EXPECT_EQ(ProximityEngineTester::backend(copied_engine),
            CollisionBackend::kCoal);
  EXPECT_EQ(copied_engine.num_geometries(), 2);

  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  EXPECT_EQ(ProximityEngineTester::backend(*ad_engine),
            CollisionBackend::kCoal);
  EXPECT_EQ(ad_engine->num_geometries(), 2);
}

/* Engines on different backends are not copies of each other, however alike
 their contents. */
TEST_F(CoalEngineTest, DeepCopyIsBackendAware) {
  ProximityEngine<double> fcl_engine{CollisionBackend::kFcl};
  ProximityEngine<double> coal_engine{CollisionBackend::kCoal};
  for (ProximityEngine<double>* engine : {&fcl_engine, &coal_engine}) {
    const ProximityProperties props;
    engine->AddDynamicGeometry(Sphere(1.0), RigidTransformd{}, id_A_, props);
    engine->AddDynamicGeometry(Sphere(1.0), RigidTransformd{}, id_B_, props);
    engine->UpdateWorldPoses(X_WGs_);
  }
  EXPECT_FALSE(ProximityEngineTester::IsDeepCopy(coal_engine, fcl_engine));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
