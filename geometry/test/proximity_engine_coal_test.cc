#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
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

/* Drives the convex hull construction path, which a primitive Box does not:
 the engine represents Box as a Box, and only Mesh and Convex go through
 ImplementFromConvexHull(). quad_cube.obj is a 2x2x2 cube of six quad faces.

 What this pins down is the hull's *geometry* -- the vertices and, crucially,
 the vertex count, which is a real D4 difference: Coal's Convex constructor
 takes the count where FCL's infers it, so a transposed or wrong argument
 here is a hard crash rather than a slightly wrong number.

 It does not test the fan-triangulation. Coal stores the polygon list but
 derives nothing from it as Drake constructs it -- the support function reads
 the vertex array -- so these assertions are unchanged by a triangle list
 that is wound backwards or empty. MakeFaces() is tested directly instead;
 see geometry/proximity/test/coal_backend_test.cc.  */
TEST_F(CoalEngineTest, ConvexHullGeometry) {
  const std::string path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  ProximityEngine<double> engine{CollisionBackend::kCoal};
  const GeometryId id_sphere = GeometryId::get_new_id();
  const GeometryId id_convex = GeometryId::get_new_id();
  const ProximityProperties props;
  engine.AddDynamicGeometry(Sphere(0.1), RigidTransformd{}, id_sphere, props);
  engine.AddDynamicGeometry(Convex(path, 1.0), RigidTransformd{}, id_convex,
                            props);

  // The hull is the cube [-1, 1]^3. Probe it along each axis in turn, so a
  // single mis-wound face cannot hide behind the symmetry of one direction.
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {1.0, -1.0}) {
      SCOPED_TRACE(fmt::format("axis {} sign {}", axis, sign));
      Vector3d p_WS = Vector3d::Zero();
      p_WS[axis] = sign * 1.5;
      const unordered_map<GeometryId, RigidTransformd> X_WGs{
          {id_convex, RigidTransformd{}}, {id_sphere, RigidTransformd{p_WS}}};
      engine.UpdateWorldPoses(X_WGs);

      const vector<SignedDistancePair<double>> pairs =
          engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
      ASSERT_EQ(pairs.size(), 1);
      // Cube face at 1.0, sphere surface at 1.5 - 0.1 = 1.4.
      EXPECT_NEAR(pairs[0].distance, 0.4, 1e-6);

      // The witness point on the cube must lie on the face we approached.
      const bool convex_is_A = pairs[0].id_A == id_convex;
      const Vector3d& p_on_cube = convex_is_A ? pairs[0].p_ACa : pairs[0].p_BCb;
      EXPECT_NEAR(p_on_cube[axis], sign * 1.0, 1e-6);
    }
  }

  // And a penetrating configuration, which is where a bad hull shows up as a
  // wrong depth rather than a wrong distance.
  const unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_convex, RigidTransformd{}},
      {id_sphere, RigidTransformd{Vector3d{1.05, 0, 0}}}};
  engine.UpdateWorldPoses(X_WGs);
  const vector<PenetrationAsPointPair<double>> pairs =
      engine.ComputePointPairPenetration(X_WGs);
  ASSERT_EQ(pairs.size(), 1);
  // Sphere centre 0.05 outside the face, radius 0.1, so 0.05 of overlap.
  EXPECT_NEAR(pairs[0].depth, 0.05, 1e-6);
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
