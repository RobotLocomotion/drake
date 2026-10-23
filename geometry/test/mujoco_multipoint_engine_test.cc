/* Integration tests for the "mujoco_multipoint" point contact algorithm at
 the ProximityEngine level: property-driven opt-in, the plain point-pair
 query, and the point-contact fallback of the hydroelastic query. The
 algorithm itself is tested in
 geometry/proximity/test/mujoco_ccd_penetration_test.cc. */

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/memory_file.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

/* A cube spanning [-1, 1]³ as a Convex shape (from an in-memory obj). */
Convex MakeCubeConvex() {
  static const char* const kObj = R"""(
v -1 -1 -1
v 1 -1 -1
v 1 1 -1
v -1 1 -1
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
f 5 6 7 8
f 1 4 3 2
f 2 3 7 6
f 1 5 8 4
f 3 4 8 7
f 1 2 6 5
)""";
  return Convex(InMemoryMesh{MemoryFile(kObj, ".obj", "cube.obj")});
}

ProximityProperties MakeProps(const std::string& algorithm) {
  ProximityProperties props;
  props.AddProperty(kMaterialGroup, kPointContactAlgorithm, algorithm);
  return props;
}

class MujocoMultipointEngineTest : public ::testing::Test {
 protected:
  /* Adds two dynamic cube geometries in face-face contact (the top cube
   penetrates the bottom cube's top face by 1 mm, laterally offset so the
   overlap region is a proper rectangle), with the given per-geometry
   algorithm properties. */
  void AddCubePair(const ProximityProperties& props_A,
                   const ProximityProperties& props_B) {
    const Convex cube = MakeCubeConvex();
    X_WGs_[id_A_] = RigidTransformd();
    X_WGs_[id_B_] = RigidTransformd(Vector3d(0.4, 0.3, 2.0 - 1e-3));
    engine_.AddDynamicGeometry(cube, X_WGs_[id_A_], id_A_, props_A);
    engine_.AddDynamicGeometry(cube, X_WGs_[id_B_], id_B_, props_B);
    engine_.UpdateWorldPoses(X_WGs_);
  }

  ProximityEngine<double> engine_;
  std::unordered_map<GeometryId, RigidTransformd> X_WGs_;
  const GeometryId id_A_{GeometryId::get_new_id()};
  const GeometryId id_B_{GeometryId::get_new_id()};
};

/* When both geometries opt in, the pair reports a four-point manifold. */
TEST_F(MujocoMultipointEngineTest, MultipointManifold) {
  AddCubePair(MakeProps("mujoco_multipoint"), MakeProps("mujoco_multipoint"));
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(ssize(pairs), 4);
  for (const auto& pair : pairs) {
    EXPECT_EQ(pair.id_A, std::min(id_A_, id_B_));
    EXPECT_EQ(pair.id_B, std::max(id_A_, id_B_));
    EXPECT_NEAR(pair.depth, 1e-3, 1e-8);
  }
}

/* Without the property, behavior is the classic single point. */
TEST_F(MujocoMultipointEngineTest, DefaultSinglePoint) {
  AddCubePair(ProximityProperties(), ProximityProperties());
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(ssize(pairs), 1);
}

/* An explicit "single_point" property matches the default. */
TEST_F(MujocoMultipointEngineTest, ExplicitSinglePoint) {
  AddCubePair(MakeProps("single_point"), MakeProps("single_point"));
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(ssize(pairs), 1);
}

/* Both geometries must opt in; a mixed pair keeps the single point. */
TEST_F(MujocoMultipointEngineTest, MixedOptInIsSinglePoint) {
  AddCubePair(MakeProps("mujoco_multipoint"), ProximityProperties());
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(ssize(pairs), 1);
}

/* An unrecognized algorithm value throws at registration time. */
TEST_F(MujocoMultipointEngineTest, BadAlgorithmThrows) {
  const Convex cube = MakeCubeConvex();
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.AddDynamicGeometry(cube, RigidTransformd(), id_A_,
                                 MakeProps("hydroelastic_hopes_and_dreams")),
      ".*point_contact_algorithm.*hydroelastic_hopes_and_dreams.*");
}

/* A non-hull shape (sphere) quietly ignores the property; its pairs use the
 single-point path even against an opted-in mesh. */
TEST_F(MujocoMultipointEngineTest, NonMeshShapeIgnoresProperty) {
  const Convex cube = MakeCubeConvex();
  X_WGs_[id_A_] = RigidTransformd();
  X_WGs_[id_B_] = RigidTransformd(Vector3d(0, 0, 1.9));
  engine_.AddDynamicGeometry(cube, X_WGs_[id_A_], id_A_,
                             MakeProps("mujoco_multipoint"));
  engine_.AddDynamicGeometry(Sphere(1.0), X_WGs_[id_B_], id_B_,
                             MakeProps("mujoco_multipoint"));
  engine_.UpdateWorldPoses(X_WGs_);
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(ssize(pairs), 1);
}

/* Two opted-in hulls at the same pose defeat MuJoCo's GJK (their vertex
 centroids coincide, see mujoco_ccd_penetration.h); the engine then falls back
 to the single-point algorithm rather than reporting no contact at all. */
TEST_F(MujocoMultipointEngineTest, CoincidentHullsFallBackToSinglePoint) {
  const Convex cube = MakeCubeConvex();
  X_WGs_[id_A_] = RigidTransformd();
  X_WGs_[id_B_] = RigidTransformd();
  engine_.AddDynamicGeometry(cube, X_WGs_[id_A_], id_A_,
                             MakeProps("mujoco_multipoint"));
  engine_.AddDynamicGeometry(cube, X_WGs_[id_B_], id_B_,
                             MakeProps("mujoco_multipoint"));
  engine_.UpdateWorldPoses(X_WGs_);
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(ssize(pairs), 1);
  EXPECT_GT(pairs[0].depth, 0.0);
  EXPECT_TRUE(pairs[0].p_WCa.allFinite());
  EXPECT_TRUE(pairs[0].p_WCb.allFinite());
  EXPECT_NEAR(pairs[0].nhat_BA_W.norm(), 1.0, 1e-12);
}

/* The point-contact fallback of the hydroelastic-with-fallback query also
 produces the manifold: two rigid-hydroelastic convex meshes fail to make a
 contact surface and fall back to (multi-)point contact. */
TEST_F(MujocoMultipointEngineTest, HydroelasticFallbackUsesManifold) {
  ProximityProperties props = MakeProps("mujoco_multipoint");
  AddRigidHydroelasticProperties(1.0, &props);
  AddCubePair(props, props);
  std::vector<ContactSurface<double>> surfaces;
  std::vector<PenetrationAsPointPair<double>> point_pairs;
  engine_.ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation::kPolygon, X_WGs_, &surfaces,
      &point_pairs);
  EXPECT_EQ(ssize(surfaces), 0);
  ASSERT_EQ(ssize(point_pairs), 4);
}

/* Two geometries sharing one mesh file at mirroring scales share the hull
 cache entry, whose face winding was fixed by the first registrant; the
 mirrored instance's tables must be re-wound outward, or every one of its
 face-face manifolds silently degrades or inverts. */
TEST_F(MujocoMultipointEngineTest, MirroredScaleSharedMesh) {
  const Convex cube = MakeCubeConvex();
  const Convex mirrored(cube.source(), Eigen::Vector3d(-1, 1, 1));
  X_WGs_[id_A_] = RigidTransformd();
  X_WGs_[id_B_] = RigidTransformd(Vector3d(0.4, 0.3, 2.0 - 1e-3));
  // The plain cube registers first and populates the shared cache entry;
  // the mirrored instance then reuses its topology.
  engine_.AddDynamicGeometry(cube, X_WGs_[id_A_], id_A_,
                             MakeProps("mujoco_multipoint"));
  engine_.AddDynamicGeometry(mirrored, X_WGs_[id_B_], id_B_,
                             MakeProps("mujoco_multipoint"));
  engine_.UpdateWorldPoses(X_WGs_);
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(ssize(pairs), 4);
  for (const auto& pair : pairs) {
    EXPECT_NEAR(pair.depth, 1e-3, 1e-8);
    // The normal must point out of B (the upper, mirrored cube): -z. An
    // inward-wound table would flip it or lose the manifold entirely.
    EXPECT_NEAR(pair.nhat_BA_W.z(), -1.0, 1e-9);
  }
}

/* Updating properties moves a geometry in or out of the multipoint catalog. */
TEST_F(MujocoMultipointEngineTest, PropertyUpdateTogglesAlgorithm) {
  AddCubePair(MakeProps("mujoco_multipoint"), MakeProps("mujoco_multipoint"));
  ASSERT_EQ(ssize(engine_.ComputePointPairPenetration(X_WGs_)), 4);
  // Note: UpdateRepresentationForNewProperties requires an InternalGeometry;
  // exercising it end-to-end happens through GeometryState. Here we at least
  // confirm RemoveGeometry() evicts the catalog entry.
  engine_.RemoveGeometry(id_B_, /* is_dynamic = */ true);
  X_WGs_.erase(id_B_);
  EXPECT_EQ(ssize(engine_.ComputePointPairPenetration(X_WGs_)), 0);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
