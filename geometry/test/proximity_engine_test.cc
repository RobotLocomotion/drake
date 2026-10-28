#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <ranges>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/ranges.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/proximity/mesh_distance_boundary.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
// Compare witness pair. Note that we can switch body A with body B in one pair,
// and the comparison result would be the same.
template <typename T>
void CompareSignedDistancePair(const SignedDistancePair<T>& pair,
                               const SignedDistancePair<T>& pair_expected,
                               double tol) {
  using std::abs;
  EXPECT_TRUE(abs(pair.distance - pair_expected.distance) < tol);
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
  static math::RigidTransformd GetX_WG(GeometryId id, bool is_dynamic,
                                       const ProximityEngine<T>& engine) {
    unused(is_dynamic);
    return engine.GetX_WG(id);
  }

  template <typename T>
  static bool is_inactive_dynamic_stale(const ProximityEngine<T>& engine) {
    return engine.is_inactive_dynamic_stale();
  }

  template <typename T>
  static HydroelasticType hydroelastic_type(GeometryId id,
                                            const ProximityEngine<T>& engine) {
    return engine.hydroelastic_geometries().hydroelastic_type(id);
  }

  template <typename T>
  static const fcl::CollisionObjectd* GetCollisionObject(
      const ProximityEngine<T>& engine, GeometryId id) {
    void* erase_ptr = engine.GetCollisionObject(id);
    if (erase_ptr != nullptr) {
      // API promises that this is a safe cast.
      return static_cast<const fcl::CollisionObjectd*>(erase_ptr);
    }
    return nullptr;
  }

  template <typename T>
  static const TriangleSurfaceMesh<double>* get_mesh_distance_boundary(
      const ProximityEngine<T>& engine, GeometryId id) {
    return engine.mesh_distance_boundary(id);
  }

  // Returns the number of top-level file entries in the convex hull cache.
  static int convex_hull_cache_file_entries(
      const ProximityEngine<double>& engine) {
    return engine.convex_hull_cache_file_entries();
  }

  // Returns the total number of scaled_hulls sub-entries across all file
  // entries in the convex hull cache.
  static int convex_hull_cache_hull_entries(
      const ProximityEngine<double>& engine) {
    return engine.convex_hull_cache_hull_entries();
  }

  // Returns true if the given geometry id is in the reverse map AND its
  // recorded (file_key, scale_key) refer to a valid cache entry.
  static bool geometry_hull_key_valid(const ProximityEngine<double>& engine,
                                      GeometryId id) {
    return engine.geometry_hull_key_valid(id);
  }

  // Returns true if the given geometry id is absent from the reverse map.
  static bool geometry_hull_key_absent(const ProximityEngine<double>& engine,
                                       GeometryId id) {
    return engine.geometry_hull_key_absent(id);
  }

  // Returns true if every entry currently in geometry_to_hull_key points to
  // a valid (file_key, scale_key) pair in convex_hull_cache.
  static bool geometry_hull_reverse_map_consistent(
      const ProximityEngine<double>& engine) {
    return engine.geometry_hull_reverse_map_consistent();
  }
};

namespace {

using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrixd;

using Eigen::AngleAxisd;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using std::make_shared;
using std::make_unique;
using std::shared_ptr;
using std::unordered_map;
using std::vector;

using symbolic::Expression;

// Tests for manipulating the population of the proximity engine.

class ProximityEngineTests : public ::testing::Test {
 protected:
  using Tester = ProximityEngineTester;
  using V3 = Vector3d;

  // Adds the given shape as a *dynamic* geometry at the given (optional)
  // position to the test's engine, registering the pose with the global pose
  // lookup table.
  GeometryId AddDynamic(const Shape& shape,
                        const std::optional<Vector3d>& p_WG = std::nullopt,
                        const ProximityProperties& props = {}) {
    const GeometryId id = GeometryId::get_new_id();
    const RigidTransformd X_WG(p_WG.has_value() ? *p_WG : Vector3d::Zero());
    X_WGs_.insert({id, X_WG});
    engine_.AddDynamicGeometry(shape, X_WG, id, props);
    return id;
  }

  // Adds the given shape as an *anchored* geometry at the given (optional)
  // position to the test's engine, registering the pose with the global pose
  // lookup table.
  GeometryId AddAnchored(const Shape& shape,
                         const std::optional<Vector3d>& p_WG = std::nullopt,
                         const ProximityProperties& props = {}) {
    const GeometryId id = GeometryId::get_new_id();
    const RigidTransformd X_WG(p_WG.has_value() ? *p_WG : Vector3d::Zero());
    X_WGs_.insert({id, X_WG});
    engine_.AddAnchoredGeometry(shape, X_WG, id, props);
    return id;
  }

  // Wrapper for declaring a collision filter between all of the listed ids.
  // If `is_temporary` is `true`, a FilterId will be returned that can
  // subsequently be removed via the RemoveDeclaration() API.
  std::optional<FilterId> ExcludeCollisionsWithin(
      std::initializer_list<GeometryId> ids, bool is_temporary = false) {
    // Note: we *pass* an empty GeometrySet to ExcludeWithin because,
    // ultimately the extract_ids lambda function will return the actual ids.
    // It's just a simplifying trick for the test.
    if (is_temporary) {
      return engine_.collision_filter().ApplyTransient(
          CollisionFilterDeclaration().ExcludeWithin(GeometrySet()),
          [&ids](const GeometrySet&, CollisionFilterScope) {
            return std::unordered_set<GeometryId>{ids};
          });
    }
    engine_.collision_filter().Apply(
        CollisionFilterDeclaration().ExcludeWithin(GeometrySet()),
        [&ids](const GeometrySet&, CollisionFilterScope) {
          return std::unordered_set<GeometryId>{ids};
        },
        /* is_invariant= */ false);
    return std::nullopt;
  }

  // Wrappers for deactivating/reactivating the listed ids. Mutating the
  // engine's filter requires forwarding the resulting active-status change to
  // the engine so its broadphase reflects it; in the SceneGraph pipeline
  // CollisionFilterManager does this. These helpers replicate that contract.
  void Deactivate(std::initializer_list<GeometryId> ids) {
    SetActiveStatus(ids, /* active= */ false);
  }

  void Activate(std::initializer_list<GeometryId> ids) {
    SetActiveStatus(ids, /* active= */ true);
  }

  void SetActiveStatus(std::initializer_list<GeometryId> ids, bool active) {
    engine_.collision_filter().SetActiveStatus(
        GeometrySet(), active,
        [&ids](const GeometrySet&, CollisionFilterScope) {
          return std::unordered_set<GeometryId>{ids};
        },
        [this](const internal::CollisionFilter::ActiveStatusChange& change) {
          engine_.ApplyActiveStatusChange(change);
        });
  }

  ProximityEngine<double> engine_;
  unordered_map<GeometryId, RigidTransform<double>> X_WGs_;
};

// AnchorGeometries() moves a geometry's fcl object from the dynamic
// collections into the anchored ones, and marks it anchored in the filter.
// The two must happen together: the engine routes object lookup, removal and
// pose updates by which collection holds the object.
TEST_F(ProximityEngineTests, AnchorGeometries) {
  const Sphere sphere{0.5};
  const GeometryId dynamic_id = AddDynamic(sphere, V3{0, 0, 0});
  const GeometryId promoted_id = AddDynamic(sphere, V3{4, 0, 0});
  const GeometryId declared_id = AddAnchored(sphere, V3{8, 0, 0});
  ASSERT_EQ(engine_.num_dynamic(), 2);
  ASSERT_EQ(engine_.num_anchored(), 1);
  ASSERT_FALSE(engine_.collision_filter().IsAnchored(promoted_id));

  engine_.AnchorGeometries({promoted_id});

  EXPECT_EQ(engine_.num_dynamic(), 1);
  EXPECT_EQ(engine_.num_anchored(), 2);
  EXPECT_TRUE(engine_.collision_filter().IsAnchored(promoted_id));
  // The move itself does not disturb the object's pose.
  EXPECT_TRUE(CompareMatrices(
      Tester::GetX_WG(promoted_id, false, engine_).translation(), V3{4, 0, 0}));

  // The promoted geometry can no longer collide with other anchored geometry,
  // but still can with what remains dynamic.
  EXPECT_FALSE(
      engine_.collision_filter().CanCollideWith(promoted_id, declared_id));
  EXPECT_TRUE(
      engine_.collision_filter().CanCollideWith(promoted_id, dynamic_id));

  // Ids that are already anchored are ignored rather than rejected.
  DRAKE_EXPECT_NO_THROW(engine_.AnchorGeometries({promoted_id, declared_id}));
  EXPECT_EQ(engine_.num_anchored(), 2);

  // Removal routes by actual membership, so the stale "is it dynamic?" answer
  // the caller started with cannot misdirect it.
  DRAKE_EXPECT_NO_THROW(engine_.RemoveGeometry(promoted_id));
  EXPECT_EQ(engine_.num_anchored(), 1);
}

// Anchoring an *inactive* geometry: two independent properties (anchored,
// inactive) span three trees, and this combination is the one that is easy to
// leave undefined. The geometry leaves the inactive dynamic partition for the
// anchored tree -- it is simply no longer culled -- and the filter's inactive
// set goes on blocking its pairs.
TEST_F(ProximityEngineTests, AnchorInactiveGeometry) {
  const Sphere sphere{0.5};
  const GeometryId other_id = AddDynamic(sphere, V3{0, 0, 0});
  const GeometryId promoted_id = AddDynamic(sphere, V3{0.5, 0, 0});

  Deactivate({promoted_id});
  ASSERT_EQ(engine_.num_inactive_dynamic(), 1);
  ASSERT_TRUE(engine_.IsInactiveDynamic(promoted_id));
  // The two spheres overlap, but the inactive one collides with nothing.
  ASSERT_FALSE(
      engine_.collision_filter().CanCollideWith(promoted_id, other_id));

  engine_.AnchorGeometries({promoted_id});

  EXPECT_EQ(engine_.num_inactive_dynamic(), 0);
  EXPECT_FALSE(engine_.IsInactiveDynamic(promoted_id));
  EXPECT_EQ(engine_.num_anchored(), 1);
  // Still blocked -- by the filter's inactive set, which promotion left alone.
  EXPECT_FALSE(
      engine_.collision_filter().CanCollideWith(promoted_id, other_id));
  EXPECT_EQ(engine_.ComputePointPairPenetration(X_WGs_).size(), 0u);

  // Reactivating it now has nothing to migrate, and unblocks the pair.
  Activate({promoted_id});
  EXPECT_EQ(engine_.num_inactive_dynamic(), 0);
  EXPECT_TRUE(engine_.collision_filter().CanCollideWith(promoted_id, other_id));
  EXPECT_EQ(engine_.ComputePointPairPenetration(X_WGs_).size(), 1u);
}

// RefreshAnchoredPoses() re-poses anchored geometry and refits the anchored
// tree. Anchored poses change only when a parameter does, so this is driven on
// demand rather than every step.
TEST_F(ProximityEngineTests, RefreshAnchoredPoses) {
  const Sphere sphere{0.5};
  const GeometryId dynamic_id = AddDynamic(sphere, V3{0, 0, 0});
  const GeometryId promoted_id = AddDynamic(sphere, V3{10, 0, 0});
  engine_.AnchorGeometries({promoted_id});
  ASSERT_EQ(engine_.ComputePointPairPenetration(X_WGs_).size(), 0u);

  // Move the anchored geometry onto the dynamic one, as changing a weld
  // parameter would. Until the refresh, the engine still sees it at the old
  // location.
  X_WGs_.at(promoted_id) = RigidTransformd(V3{0.5, 0, 0});
  engine_.UpdateWorldPoses(X_WGs_);
  EXPECT_EQ(engine_.ComputePointPairPenetration(X_WGs_).size(), 0u);

  engine_.RefreshAnchoredPoses(X_WGs_);
  EXPECT_TRUE(CompareMatrices(
      Tester::GetX_WG(promoted_id, false, engine_).translation(),
      V3{0.5, 0, 0}));
  const auto pairs = engine_.ComputePointPairPenetration(X_WGs_);
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(SortedPair<GeometryId>(pairs[0].id_A, pairs[0].id_B),
            SortedPair<GeometryId>(dynamic_id, promoted_id));
}

// Test simple addition of dynamic and anchored geometry. Confirms that the
// added geometries are reflected in the bookkeeping.
TEST_F(ProximityEngineTests, AddGeometry) {
  const Sphere sphere{0.5};
  const Vector3d p_WA{1, 2, 3};
  const GeometryId id_1 = AddAnchored(sphere, p_WA);
  EXPECT_TRUE(
      CompareMatrices(Tester::GetX_WG(id_1, false, engine_).GetAsMatrix34(),
                      RigidTransformd(p_WA).GetAsMatrix34()));

  const Vector3d p_WD{-1, -2, -3};
  const GeometryId id_2 = AddDynamic(sphere, p_WD);
  EXPECT_TRUE(
      CompareMatrices(Tester::GetX_WG(id_2, true, engine_).GetAsMatrix34(),
                      RigidTransformd(p_WD).GetAsMatrix34()));

  EXPECT_EQ(engine_.num_geometries(), 2);
  EXPECT_EQ(engine_.num_anchored(), 1);
  EXPECT_EQ(engine_.num_dynamic(), 1);
}

// When compliant hydroelastic geometries have a positive margin value,
// ProximityEngine must inflate the AABB that fcl uses. This test confirms that
// inflations happen as expected. Our expectations are as follows:
//
//    1. Geometry that has no hydro representation does not get inflated.
//    2. Geometry that has a *rigid* representation does not get inflated.
//    3. Geometry that has a *compliant* representation gets inflated.
//       - Primitives get inflated by the margin amount in each direction.
//       - Meshes get inflated an arbitrary amount in each direction at least as
//         large as the margin value.
TEST_F(ProximityEngineTests, HydroelasticAabbInflation) {
  // All of the geometries will have a scale comparable to edge_length, so that
  // the mesh creation is as cheap as possible. The exception is Mesh
  // geometry since we have no re-meshing.
  const double edge_length = 0.5;
  const double E = 1e8;  // Elastic modulus.
  const double kMarginValue = 0.0625;

  // We'll add the margin property to all sets of properties -- it will be
  // ignored for all geometries, except those with compliant hydro.
  ProximityProperties no_hydro_properties;
  no_hydro_properties.AddProperty(kHydroGroup, kMargin, kMarginValue);

  ProximityProperties compliant_properties(no_hydro_properties);
  AddCompliantHydroelasticProperties(edge_length, E, &compliant_properties);

  ProximityProperties rigid_properties(no_hydro_properties);
  AddRigidHydroelasticProperties(edge_length, &rigid_properties);

  // We'll use a Box as representative of all primitives.
  const Box box(1, 1, 1);

  const Mesh mesh{FindResourceOrThrow("drake/geometry/test/octahedron.obj")};
  const Convex convex(mesh.source());
  const Vector3d mesh_min(-1, -1, -1.414213562373);
  const Vector3d mesh_max = -mesh_min;

  const Mesh vol_mesh(
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk"));
  const Vector3d vol_min(0, 0, 0);
  const Vector3d vol_max(1, 1, 1);

  struct TestCase {
    const Shape* shape;
    ProximityProperties properties;
    Vector3d expected_min;
    Vector3d expected_max;
    // If true, the bounding box extents must match the expected extents.
    // If false, they must be bigger (i.e., the expected box must lie completely
    // within the actual box).
    bool expect_exact{};
    std::string description;
  };

  vector<TestCase> cases{
      {&box, no_hydro_properties, Vector3d::Constant(-0.5),
       Vector3d::Constant(0.5), true, "No hydro primitive -> no inflation"},
      {&mesh, no_hydro_properties, mesh_min, mesh_max, true,
       "No hydro mesh -> no inflation"},
      {&convex, no_hydro_properties, mesh_min, mesh_max, true,
       "No hydro convex -> no inflation"},
      {&vol_mesh, no_hydro_properties, vol_min, vol_max, true,
       "No hydro volume mesh -> no inflation"},
      {&box, rigid_properties, Vector3d::Constant(-0.5),
       Vector3d::Constant(0.5), true, "Rigid-> no inflation"},
      {&box, compliant_properties, Vector3d::Constant(-0.5 - kMarginValue),
       Vector3d::Constant(0.5 + kMarginValue), true,
       "Compliant primitive-> exact inflation"},
      {&mesh, compliant_properties, mesh_min - Vector3d::Constant(kMarginValue),
       mesh_max + Vector3d::Constant(kMarginValue), false,
       "Compliant mesh -> inflation exceeds minimum"},
      {&convex, compliant_properties,
       mesh_min - Vector3d::Constant(kMarginValue),
       mesh_max + Vector3d::Constant(kMarginValue), false,
       "Compliant convex -> inflation exceeds minimum"},
      {&vol_mesh, compliant_properties,
       vol_min - Vector3d::Constant(kMarginValue),
       vol_max + Vector3d::Constant(kMarginValue), false,
       "Compliant vol mesh -> inflation exceeds minimum"},
  };

  for (const auto& test_case : cases) {
    const GeometryId id =
        AddDynamic(*test_case.shape, {}, test_case.properties);
    const auto* fcl = Tester::GetCollisionObject(engine_, id);
    DRAKE_DEMAND(fcl != nullptr);
    const auto& aabb = fcl->collisionGeometry()->aabb_local;
    if (test_case.expect_exact) {
      EXPECT_TRUE(CompareMatrices(aabb.min_, test_case.expected_min))
          << test_case.description;
      EXPECT_TRUE(CompareMatrices(aabb.max_, test_case.expected_max))
          << test_case.description;
    } else {
      for (int i = 0; i < 3; ++i) {
        EXPECT_LT(aabb.min_[i], test_case.expected_min[i])
            << test_case.description;
        EXPECT_GT(aabb.max_[i], test_case.expected_max[i])
            << test_case.description;
      }
    }
  }
}

// After updating the margin property of a registered shape, we need to make
// sure that the new margin is accounted for.
// We place two spheres in near proximity such that their marginal regions
// overlap.
//
//              │
//            ○○○○○        .....
//          ○○  │  ○○    ..     ..
//        ○○   ●●●   ○○..   +++   ..
//       ○   ●  │  ●  .○   +   +    .
//    ───○──●───B───●─.○──+──A──+───.──────────────
//       ○   ●  │  ●  .○   +   +    .
//        ○○   ●●●   ○○..   +++   ..
//          ○○  │  ○○    ..     ..
//            ○○○○○        .....
//              │
//
//  - Spheres A and B with radius R and margin δ.
//  - ● Boundary of B with radius R
//  - ○ Margin boundary of B with radius (R + δ).
//  - + Boundary of A with radius R
//  - . Margin boundary of A with radius (R + δ).
//
// We must register geometries with compliant hydro representations for margin
// to have an effect. However, we can *detect* the effect simply by asking for
// the collision candidates (a cheaper operation than computing hydro contact
// surfaces).
TEST_F(ProximityEngineTests, MarginAfterPropertyUpdate) {
  // All of the geometries will have a scale comparable to edge_length, so that
  // the mesh creation is as cheap as possible. The exception is Mesh
  // geometry since we have no re-meshing.
  const double kRadius = 0.5;
  const double kResHint = kRadius / 5;
  const double kE = 1e8;            // Elastic modulus.
  const double kMarginValue = 0.1;  // Large margin value for clarity.
  const double kDistance = 2 * (kRadius + kMarginValue) - 0.5 * kMarginValue;

  const Sphere sphere(kRadius);

  ProximityProperties compliant_properties;
  compliant_properties.AddProperty(kHydroGroup, kMargin, 0.0);
  AddCompliantHydroelasticProperties(kResHint, kE, &compliant_properties);

  // Shape A starts with zero margin.
  const Vector3d p_WA(kDistance, 0, 0);
  const GeometryId idA = AddDynamic(sphere, p_WA, compliant_properties);

  // Shape B gets margin -- we don't need to track the id.
  compliant_properties.UpdateProperty(kHydroGroup, kMargin, kMarginValue);
  AddAnchored(sphere, {}, compliant_properties);

  engine_.UpdateWorldPoses(X_WGs_);

  // idA has no margin; there should be no collision candidates.
  EXPECT_EQ(engine_.FindCollisionCandidates().size(), 0);

  // Now update A to get the margin. (To do so, we need a dummy InternalGeometry
  // associated with the id).
  const InternalGeometry geo_A(SourceId::get_new_id(),
                               std::make_unique<Sphere>(kRadius),
                               FrameId::get_new_id(), idA, "A", X_WGs_.at(idA));
  engine_.UpdateRepresentationForNewProperties(geo_A, compliant_properties);

  // Now they overlap and should be reported as a collision candidate.
  EXPECT_EQ(engine_.FindCollisionCandidates().size(), 1);
}

// The same test set up as MarginAfterPropertyUpdate. However, this time we
// confirm that margins survive cloning.
TEST_F(ProximityEngineTests, MarginAfterEngineClone) {
  // All of the geometries will have a scale comparable to edge_length, so that
  // the mesh creation is as cheap as possible. The exception is Mesh
  // geometry since we have no re-meshing.
  const double kRadius = 0.5;
  const double kResHint = kRadius / 5;
  const double kE = 1e8;            // Elastic modulus.
  const double kMarginValue = 0.1;  // Large margin value for clarity.
  const double kDistance = 2 * (kRadius + kMarginValue) - 0.5 * kMarginValue;

  const Sphere sphere(kRadius);
  const Vector3d p_WA(kDistance, 0, 0);

  ProximityProperties compliant_properties;
  compliant_properties.AddProperty(kHydroGroup, kMargin, kMarginValue);
  AddCompliantHydroelasticProperties(kResHint, kE, &compliant_properties);

  // Both shapes have margin.
  AddDynamic(sphere, p_WA, compliant_properties);
  AddAnchored(sphere, {}, compliant_properties);

  engine_.UpdateWorldPoses(X_WGs_);

  // The original engine reports contact.
  EXPECT_EQ(engine_.FindCollisionCandidates().size(), 1);

  // A copy reports the same.
  ProximityEngine<double> copy(engine_);
  EXPECT_EQ(copy.FindCollisionCandidates().size(), 1);
}

// Tests that registering the same mesh source multiple times reuses the same
// underlying fcl::Convex collision geometry object (i.e. the convex hull cache
// is working).
TEST_F(ProximityEngineTests, ConvexHullCacheDuplicatesShareGeometry) {
  const std::string path_a =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  const std::string path_b =
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj");

  // Add the shape (dynamic vs anchored as indicated). Returns a pointer to the
  // underlying collision geometry -- the cached quantity.
  auto add_geometry = [this](const Shape& shape, bool is_dynamic = true) {
    GeometryId id;
    if (is_dynamic) {
      id = AddDynamic(shape);
    } else {
      id = AddAnchored(shape);
    }
    const fcl::CollisionObjectd* obj = Tester::GetCollisionObject(engine_, id);
    DRAKE_DEMAND(obj != nullptr);
    return obj->collisionGeometry().get();
  };

  // We'll register one path as multiple geometry (some Mesh, some Convex).
  const fcl::CollisionGeometryd* convex_a1 = add_geometry(Convex(path_a));
  const fcl::CollisionGeometryd* convex_a2 = add_geometry(Convex(path_a));
  const fcl::CollisionGeometryd* mesh_a1 = add_geometry(Mesh(path_a));
  const fcl::CollisionGeometryd* mesh_a2 = add_geometry(Mesh(path_a));

  // Start by confirming our reference geometry isn't null.
  ASSERT_NE(convex_a1, nullptr);

  // All of the collision objects instantiated by path_a (whether Mesh or
  // Convex) should all share the same collision geometry.
  EXPECT_EQ(convex_a1, convex_a2);
  EXPECT_EQ(convex_a1, mesh_a1);
  EXPECT_EQ(convex_a1, mesh_a2);

  // We'll use a second mesh to show that not all meshes get cached the same.
  // We'll also use the second mesh to show that anchored/dynamic doesn't
  // matter.
  const fcl::CollisionGeometryd* convex_b = add_geometry(Convex(path_b));
  const fcl::CollisionGeometryd* mesh_b =
      add_geometry(Mesh(path_b), /* is_dynamic= */ false);

  // From a different path, we should get a different geometry.
  EXPECT_NE(convex_b, nullptr);
  EXPECT_NE(convex_a1, convex_b);
  EXPECT_EQ(convex_b, mesh_b);
}

// Tests that the convex hull cache correctly handles multiple registrations of
// the same mesh file with different anisotropic scale factors. The cache must
// produce a distinct fcl::Convexd—with correctly-scaled vertex positions for
// each unique scale, rather than reusing the hull built for the first scale
// seen.
//
// Setup: three Convex instances all sourced from the same obj file (a 2m cube;
// unit half-extents of 1m in each axis). Each has an anisotropic scale that
// doubles exactly one axis, and each box is placed 10m along that axis from the
// origin.
//
//   Instance  Position        Scale      Active half-extent  Expected distance
//      A      (-10,  0,  0)  (2, 1, 1)   x: 2 m              8.0 m
//      B      ( 0, -10,  0)  (1, 2, 1)   y: 2 m              8.0 m
//      C      ( 0,  0, -10)  (1, 1, 2)   z: 2 m              8.0 m
//
// If the scale is wrong, things could fail in two ways: at the broadphase and
// at the narrowphase. To test the broadphase, we introduce a query threshold
// that is farther than the correct distance (8.0m) but closer than the distance
// associated with an unscaled box (9.0m). If the box hasn't been scaled
// properly, the broadphase will prune it and there will be no result for the
// geometry.
//
// For the narrow phase, we simply confirm that the reported distances are all
// as expected.
TEST_F(ProximityEngineTests, ConvexHullCacheScaleIsRespected) {
  const std::string path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  // Each box is placed 10m along a different axis and scaled to double only
  // that axis' half-extent, so the distance from the origin to each box's
  // nearest face is exactly 10 - 2 = 8 m.
  const Vector3d p_WA(-10, 0, 0);
  const Vector3d p_WB(0, -10, 0);
  const Vector3d p_WC(0, 0, -10);

  const GeometryId id_A = AddAnchored(Convex(path, Vector3d(2, 1, 1)), p_WA);
  const GeometryId id_B = AddAnchored(Convex(path, Vector3d(1, 2, 1)), p_WB);
  const GeometryId id_C = AddAnchored(Convex(path, Vector3d(1, 1, 2)), p_WC);

  // Threshold chosen between the correct nearest-face distance (8.0 m) and the
  // nearest distance produced by a wrong-scale (x-doubled) hull placed at
  // y/z = -10 m (which would be 9.0 m).
  const double threshold = 8.5;
  const auto results =
      engine_.ComputeSignedDistanceToPoint(Vector3d::Zero(), X_WGs_, threshold);

  // All three geometries must appear in results with their correct distances.
  // If any cache entry returned the wrong scale, the corresponding geometry's
  // FCL AABB would be too far from the origin to pass the threshold and it
  // would be missing from results.
  ASSERT_EQ(results.size(), 3);
  std::unordered_map<GeometryId, double> dist_by_id;
  for (const auto& r : results) {
    dist_by_id[r.id_G] = r.distance;
  }
  constexpr double kTol = 1e-14;
  EXPECT_NEAR(dist_by_id.at(id_A), 8.0, kTol);  // confirms x-scale of Box A
  EXPECT_NEAR(dist_by_id.at(id_B), 8.0, kTol);  // confirms y-scale of Box B
  EXPECT_NEAR(dist_by_id.at(id_C), 8.0, kTol);  // confirms z-scale of Box C
}

// Tests that Convex shapes sourced from the same file but registered with
// different margins receive distinct fcl::Convexd objects (as each Convexd
// stores a local AABB encompassing the geometry *and* its margin). The
// declarations can't interfere with each other.
//
// We'll declare three convex geometries (A, B, C in that order). A and C have
// zero margin and B has a non-zero margin. The boxes will be arrayed along an
// axis with a gap between them slightly smaller than the non-zero margin.
// They are arrayed in the order: A C B.
//
//                     ┌┄┄┄┄┄┄┄┄┄┄┄┐
//        ┏━━━━━┓ ┏━━━━┿┓ ┏━━━━━┓  ┊
//        ┃  A  ┃ ┃  C ┊┃ ┃  B  ┃  ┊
//        ┗━━━━━┛ ┗━━━━┿┛ ┗━━━━━┛  ┊ <-- B with margin
//                     └┄┄┄┄┄┄┄┄┄┄┄┘
//
// Failure modes we're looking to detect:
//
//   1. Subsequent registrations stomp on previous registrations.
//      In this case, the overlap between B and C would be lost (C's last
//      zero-margin registration would replace B's margin).
//   2. Subsequent registrations don't get a new margin.
//      In this case, the overlap between B and C would be lost (this time
//      because B -- and C -- simply picked up the original zero-margin AABB).
TEST_F(ProximityEngineTests, ConvexHullCacheMarginIsRespected) {
  const double kResHint = 0.5;
  const double kE = 1e8;
  const double kMarginValue = 0.1;
  const double kBoxExtent = 2.0;

  // Spatial order A C B; registration order A B C.
  const double kOffset = kBoxExtent + kMarginValue / 2;
  const Vector3d p_WA(-kOffset, 0, 0);
  const Vector3d p_WC(0, 0, 0);
  const Vector3d p_WB(kOffset, 0, 0);

  ProximityProperties props_no_margin;
  props_no_margin.AddProperty(kHydroGroup, kMargin, 0.0);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_no_margin);

  ProximityProperties props_with_margin;
  props_with_margin.AddProperty(kHydroGroup, kMargin, kMarginValue);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_with_margin);

  const Convex convex(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"));
  const GeometryId id_A = AddDynamic(convex, p_WA, props_no_margin);
  const GeometryId id_B = AddDynamic(convex, p_WB, props_with_margin);
  const GeometryId id_C = AddDynamic(convex, p_WC, props_no_margin);

  // Call UpdateWorldPoses for the three dynamic geometries. This triggers
  // computeAABB() for B, which reads the shared geometry's aabb_local. In
  // the bug case, C's registration reset that aabb_local and B loses its
  // inflation here.
  engine_.UpdateWorldPoses(X_WGs_);

  // B's inflated AABB must overlap C; A is far away and never a candidate.
  std::vector<SortedPair<GeometryId>> candidates =
      engine_.FindCollisionCandidates();
  auto formattable_view =
      candidates | std::views::transform([](const SortedPair<GeometryId>& p) {
        return fmt::format("({}, {})", p.first(), p.second());
      });
  EXPECT_EQ(candidates.size(), 1) << fmt::format(
      "For A={}, B={}, C={}, we have the following candidates:\n{}", id_A, id_B,
      id_C, fmt::join(formattable_view, ", "));
}

// Tests that updating the margin of an existing geometry (via
// UpdateRepresentationForNewProperties()) does not corrupt the broad-phase AABB
// of another geometry registered from the same source file but with a different
// margin.
//
// We place two convex shapes near a reference sphere at the origin. With a
// zero-valued margin, the AABBs would not overlap. With the initial non-zero
// margin, they both overlap. We'll reduce the margin of one shape and show
// that it no longer overlaps, but the other still does; its margin has been
// unaffected.
TEST_F(ProximityEngineTests,
       ConvexHullCacheMarginUpdateDoesNotCorruptSiblings) {
  const double kResHint = 0.5;
  const double kE = 1e8;
  const double kSmallMargin = 0.1;   // A's margin.
  const double kLargeMargin = 0.2;   // B's margin.
  const double kSphereRadius = 0.5;  // Reference geometry radius.

  const double kBoxExtent = 2;
  const double kOffset = kBoxExtent / 2 + kSphereRadius;

  const Vector3d p_WA(-kOffset - kSmallMargin / 2, 0, 0);
  const Vector3d p_WB(kOffset + kLargeMargin / 2, 0, 0);
  const Vector3d p_WS(0, 0, 0);

  ProximityProperties props_small;
  props_small.AddProperty(kHydroGroup, kMargin, kSmallMargin);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_small);

  ProximityProperties props_large;
  props_large.AddProperty(kHydroGroup, kMargin, kLargeMargin);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_large);

  const Convex convex(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"));
  const GeometryId id_A = AddDynamic(convex, p_WA, props_small);
  // Not capturing the ids for B and S; we don't need them.
  AddDynamic(convex, p_WB, props_large);
  AddAnchored(Sphere(kSphereRadius), p_WS);
  engine_.UpdateWorldPoses(X_WGs_);

  // Both inflated AABBs must overlap the sphere => 2 candidates.
  ASSERT_EQ(engine_.FindCollisionCandidates().size(), 2);

  // Update A to zero margin. B must be unaffected.
  ProximityProperties props_zero;
  props_zero.AddProperty(kHydroGroup, kMargin, 0.0);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_zero);
  const InternalGeometry geo_A(
      SourceId::get_new_id(), std::make_unique<Convex>(convex),
      FrameId::get_new_id(), id_A, "A", X_WGs_.at(id_A));
  engine_.UpdateRepresentationForNewProperties(geo_A, props_zero);

  // UpdateWorldPoses triggers computeAABB() on all dynamic objects (A and B).
  // In the bug case, B's aabb_local was reset when A's was updated (shared
  // Convexd), so B would lose its inflation here.
  engine_.UpdateWorldPoses(X_WGs_);

  // A-sphere overlap is gone; B-sphere overlap must persist.
  EXPECT_EQ(engine_.FindCollisionCandidates().size(), 1);
}

// Tests that the convex hull cache is correctly evicted when geometries are
// removed. Exercises three eviction cases in sequence using the same initial
// population, plus a running invariant check on the reverse map.
//
// Initial setup (all from the same .obj, all dynamic):
//   G1: scale (1,1,1) — sole occupant of the (1,1,1,0) sub-entry.
//   G2: scale (2,1,1) — shares the (2,1,1,0) sub-entry with G3.
//   G3: scale (2,1,1) — shares the (2,1,1,0) sub-entry with G2.
//
// This yields 2 sub-entries under 1 file entry initially.
//
// Removal sequence and expected outcomes:
//   Remove G3: G2 still holds the (2,1,1,0) hull → no eviction.
//              2 sub-entries, 1 file entry.
//   Remove G2: cache is now sole owner of (2,1,1,0) → sub-entry evicted.
//              1 sub-entry, 1 file entry.
//   Remove G1: cache is sole owner of (1,1,1,0) → evicted; file entry empty →
//              file entry also evicted.
//              0 sub-entries, 0 file entries.
//
// After each removal, the reverse-map invariant is checked:
//   a) The removed geometry is no longer in geometry_to_hull_key.
//   b) Every remaining entry in geometry_to_hull_key has a corresponding
//      valid pair in convex_hull_cache.
TEST_F(ProximityEngineTests, ConvexHullCacheEviction) {
  const double kResHint = 0.5;
  const double kE = 1e8;

  ProximityProperties props;
  AddCompliantHydroelasticProperties(kResHint, kE, &props);

  const std::string path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  const Convex convex_unit(path, 1.0);
  const Convex convex_scaled(path, Vector3d(2, 1, 1));

  const GeometryId id_G1 = AddDynamic(convex_unit, {}, props);
  const GeometryId id_G2 = AddDynamic(convex_scaled, {}, props);
  const GeometryId id_G3 = AddDynamic(convex_scaled, {}, props);

  // Helper that checks the reverse-map invariant: every entry in
  // geometry_to_hull_key has a corresponding valid entry in convex_hull_cache,
  // and the given removed_id is absent.
  auto validate_eviction = [this](GeometryId removed_id, int expected_files,
                                  int expected_hulls,
                                  const std::string& label) {
    SCOPED_TRACE(label);
    EXPECT_TRUE(Tester::geometry_hull_key_absent(engine_, removed_id));
    EXPECT_TRUE(Tester::geometry_hull_reverse_map_consistent(engine_));
    EXPECT_EQ(Tester::convex_hull_cache_file_entries(engine_), expected_files);
    EXPECT_EQ(Tester::convex_hull_cache_hull_entries(engine_), expected_hulls);
  };

  // Confirm initial conditions.
  int expected_files = 1;
  int expected_hulls = 2;
  validate_eviction(GeometryId::get_new_id(), expected_files, expected_hulls,
                    "Fake Geometry");

  // Remove G3 — G2 still holds the (2,1,1,0) hull; no eviction expected.
  engine_.RemoveGeometry(id_G3);
  validate_eviction(id_G3, expected_files, expected_hulls, "G3 removed");

  // Remove G2 — cache is now sole owner of (2,1,1,0); sub-entry evicted.
  engine_.RemoveGeometry(id_G2);
  validate_eviction(id_G2, expected_files, --expected_hulls, "G2 removed");

  // Remove G1 — cache is sole owner of (1,1,1,0); fully evicted.
  engine_.RemoveGeometry(id_G1);
  validate_eviction(id_G1, --expected_files, --expected_hulls, "G1 removed");
}

// Tests that the signed distance field (SDF) data computed for an obj correctly
// accounts for its scale factor.
TEST_F(ProximityEngineTests, ProcessMeshSdfDataForObj) {
  const Vector3d scale(2, 3, 4);
  const Mesh mesh(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
                  scale);
  const GeometryId g_id = AddAnchored(mesh);

  const TriangleSurfaceMesh<double>* boundary_mesh =
      Tester::get_mesh_distance_boundary(engine_, g_id);

  ASSERT_NE(boundary_mesh, nullptr);
  // The cube is 2x2x2. The bounding box of the boundary mesh should be that,
  // scaled by the given scale factors.
  const auto& [_, size] = boundary_mesh->CalcBoundingBox();
  EXPECT_TRUE(CompareMatrices(size, 2 * scale));
}

// Removes geometry (dynamic and anchored) from the engine. The test creates
// a _unique_ engine instance with all dynamic or all anchored geometries.
// It is not necessary to create a mixed engine because the two geometry
// types are segregated.
TEST_F(ProximityEngineTests, RemoveGeometry) {
  // Add rigid properties so we can confirm removal of hydroelastic
  // representation. We use rigid here to make sure things get invoked and
  // rely on the implementation of hydroelastic::Geometries to distinguish
  // compliant and rigid.
  ProximityProperties props;
  AddRigidHydroelasticProperties(1.0, &props);
  auto add_geometry = [this, &props](const Shape& shape, const Vector3d& p_WG,
                                     bool is_dynamic) {
    if (is_dynamic) {
      return AddDynamic(shape, p_WG, props);
    } else {
      return AddAnchored(shape, p_WG, props);
    }
  };

  for (bool is_dynamic : {true, false}) {
    // Removing a geometry that doesn't exist, throws.
    EXPECT_THROW(engine_.RemoveGeometry(GeometryId::get_new_id()),
                 std::logic_error);

    // Populate the world with three anchored spheres located on the x-axis at
    // x = 0, 2, & 4. With radius of 0.5, they should *not* be colliding.
    Sphere sphere{0.5};
    std::vector<GeometryId> ids;
    for (int i = 0; i < 3; ++i) {
      // We're implicitly adding
      const GeometryId id = add_geometry(sphere, V3{i * 2.0, 0, 0}, is_dynamic);
      ids.push_back(id);
      EXPECT_EQ(Tester::hydroelastic_type(id, engine_),
                HydroelasticType::kRigid);
    }
    int expected_count = static_cast<int>(engine_.num_geometries());
    EXPECT_EQ(engine_.num_geometries(), expected_count);
    EXPECT_EQ(engine_.num_anchored(), is_dynamic ? 0 : expected_count);
    EXPECT_EQ(engine_.num_dynamic(), is_dynamic ? expected_count : 0);

    engine_.UpdateWorldPoses(X_WGs_);

    // Remove objects out of order from how they were added. Confirms that the
    // globals are consistent and that the geometry can only be removed once.
    for (int index : {1, 2, 0}) {
      --expected_count;
      const GeometryId remove_id = ids[index];
      engine_.RemoveGeometry(remove_id);
      EXPECT_EQ(engine_.num_geometries(), expected_count);
      EXPECT_EQ(engine_.num_anchored(), is_dynamic ? 0 : expected_count);
      EXPECT_EQ(engine_.num_dynamic(), is_dynamic ? expected_count : 0);
      EXPECT_THROW(engine_.RemoveGeometry(remove_id), std::logic_error);
      EXPECT_EQ(Tester::hydroelastic_type(remove_id, engine_),
                HydroelasticType::kUndefined);
    }
  }
}

// Tests for copy/move semantics.  ---------------------------------------------

// Tests the copy semantics of the ProximityEngine -- the copy is a complete
// copy. Each CollisionObjectd (and its transform, user data, and AABB) is
// individually duplicated, but the underlying fcl collision geometry is shared
// rather than deep-copied, since FCL geometry is treated as immutable after
// construction. As cloning makes no special effort based on geometry type, we
// can use one or two arbitrary, representative shapes for this test.
TEST_F(ProximityEngineTests, CopySemantics) {
  const Sphere sphere(0.5);
  AddAnchored(sphere);
  AddDynamic(sphere);
  AddDynamic(Convex{FindResourceOrThrow("drake/geometry/test/quad_cube.obj")});

  ProximityEngine<double> copy_construct(engine_);
  Tester::IsDeepCopy(copy_construct, engine_);

  ProximityEngine<double> copy_assign;
  copy_assign = engine_;
  Tester::IsDeepCopy(copy_assign, engine_);
}

// Tests the move semantics of the ProximityEngine -- the source is restored to
// default state.
TEST_F(ProximityEngineTests, MoveSemantics) {
  const Sphere sphere(0.5);
  AddAnchored(sphere);
  AddDynamic(sphere);
  AddDynamic(Convex{FindResourceOrThrow("drake/geometry/test/quad_cube.obj")});

  // We'll keep engine_ intact so we can compare moved instances against it.
  ProximityEngine<double> copied(engine_);

  ProximityEngine<double> move_construct(std::move(copied));
  Tester::IsDeepCopy(move_construct, engine_);
  EXPECT_EQ(copied.num_geometries(), 0);
  EXPECT_EQ(copied.num_anchored(), 0);
  EXPECT_EQ(copied.num_dynamic(), 0);

  ProximityEngine<double> move_assign;
  move_assign = std::move(move_construct);
  Tester::IsDeepCopy(move_assign, engine_);
  EXPECT_EQ(move_construct.num_geometries(), 0);
  EXPECT_EQ(move_construct.num_anchored(), 0);
  EXPECT_EQ(move_construct.num_dynamic(), 0);
}

/* ComputeSignedDistancePairwiseClosestPoints() responsibilities:
  1. Report no results for an empty engine.
  2. The max_distance parameter makes a difference (i.e., it is passed to the
     callback).
  3. Do not report anchored-anchored pairs.
  4. Report dynamic-dynamic pairs.
  5. Report dynamic-anchored pairs.
  6. Results are always ordered.
  7. Respect collision filter (filtered pairs are not reported).
  8. AutoDiff derivatives pass through successfully.
  9. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence.

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

  We set up the spheres as follows:

            ├ max_d ┤
       ···             ***             ***
     ·     ·         *     *         *     *
    ·   A1  · ← s → *   D3  * ← s → *   D4  *
    ·       ·       *       *       *       *
     ·     ·         *     *         *     *
       ···             ***             ***
        ↑        - max_d = s + δ; two spheres separated by distance s are close
        s          enough (w.r.t. max_d) to be part of the results.
        ↓        - Anchored geometries A1 and A2 are separated by s.
       ···       - Dynamic geometry D3 is separated by A1 by s, but is too far
     ·     ·       from A2 to fall under the max_d threshold.
    ·   A2  ·    - Dynamic geometry D4 is s units away from D3.
    ·       ·
     ·     ·
       ···
  */
TEST_F(ProximityEngineTests, ComputeSignedDistancePairwiseClosestPoints) {
  const Sphere sphere{0.5};
  // The separation s (between sphere surfaces).
  const double separation = 0.1;
  // Center-to-center distance for a pair separated by `separation`.
  const double d = 2 * sphere.radius() + separation;
  // max_distance large enough to include intra-cluster pairs (sep = 0.1) but
  // small enough to exclude cross-cluster pairs (clusters are 100 units apart).
  const double kMaxDist = separation + 0.1;

  auto eval_dut = [this, kMaxDist]() {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputeSignedDistancePairwiseClosestPoints(X_WGs_, kMaxDist);
  };

  // Extract the geometry ids of the results as pairs (leaves behind the
  // numerical results).
  using IdPairs = vector<std::pair<GeometryId, GeometryId>>;
  auto result_ids = [](const vector<SignedDistancePair<double>>& results) {
    IdPairs ids;
    for (const auto& result : results) {
      ids.push_back(std::make_pair(result.id_A, result.id_B));
    }
    return ids;
  };

  // (1) - empty engine reports no distances.
  ASSERT_TRUE(eval_dut().empty());

  // We'll test (2)-(5) all at the same time.
  //  (2) - max_distance prevents pairs (A2, D3), and (A*, D4).
  //  (3) - No (A1, A2) pair.
  //  (4) - Pair (D3, D4) included.
  //  (5) - Pair (A1, D3) included.
  const GeometryId anchored1 = AddAnchored(sphere, V3{0, 0, 0});
  // We don't need anchored2's id; it never gets reported.
  AddAnchored(sphere, V3{0, -d, 0});
  const GeometryId dynamic3 = AddDynamic(sphere, V3{d, 0, 0});
  const GeometryId dynamic4 = AddDynamic(sphere, V3{2 * d, 0, 0});

  // We know that geometry ids are ordered by their creation order, so we can
  // anticipate the sorted result order as well.
  const IdPairs expected{{anchored1, dynamic3}, {dynamic3, dynamic4}};
  ASSERT_EQ(result_ids(eval_dut()), expected);

  // (6) - Multiple calls produce identically ordered results (as indicated by
  // id pairs).
  ASSERT_EQ(result_ids(eval_dut()), expected);

  // (7) - Collision filter respected.
  ExcludeCollisionsWithin({anchored1, dynamic3});
  ASSERT_EQ(result_ids(eval_dut()), IdPairs({{dynamic3, dynamic4}}));

  // (8) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Seed dynamic3's translation with derivatives.
  X_WGs_ad[dynamic3] = RigidTransform<AutoDiffXd>{
      math::InitializeAutoDiff(X_WGs_.at(dynamic3).translation())};

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  const auto ad_results =
      ad_engine->ComputeSignedDistancePairwiseClosestPoints(X_WGs_ad, kMaxDist);
  // Just the one, unfiltered collision remains.
  ASSERT_EQ(ad_results.size(), 1);
  // Confirm derivatives survived -- not empty and not all zero.
  const auto& result = ad_results[0];
  const auto& derivs = result.distance.derivatives();
  ASSERT_EQ(derivs.size(), 3);
  EXPECT_FALSE(derivs.isZero());
}

/* ComputeSignedDistancePairClosestPoints() responsibilities:
  1. Given two valid ids, return a result with the correct ids.
  2. Throw if id_A does not reference a known geometry.
  3. Throw if id_B does not reference a known geometry.
  4. Evaluate the pair even if it is collision-filtered (filter is ignored).
  5. AutoDiff derivatives pass through successfully.

  Unlike the pairwise query, this API takes explicit geometry ids and bypasses
  the broadphase entirely.

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion. */
TEST_F(ProximityEngineTests, ComputeSignedDistancePairClosestPoints) {
  const Sphere sphere{0.5};
  // Place the spheres apart (not colliding) so the distance query is valid.
  const double d = 2 * sphere.radius() + 0.1;
  const GeometryId id_A = AddDynamic(sphere);
  const GeometryId id_B = AddDynamic(sphere, V3{d, 0, 0});
  const GeometryId bad_id = GeometryId::get_new_id();
  engine_.UpdateWorldPoses(X_WGs_);

  // (1) - Valid ids return a result with the correct geometry ids.
  {
    const SignedDistancePair<double> result =
        engine_.ComputeSignedDistancePairClosestPoints(id_A, id_B, X_WGs_);
    EXPECT_EQ(result.id_A, id_A);
    EXPECT_EQ(result.id_B, id_B);
  }

  // (2) - Invalid id_A throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.ComputeSignedDistancePairClosestPoints(bad_id, id_B, X_WGs_),
      fmt::format("The geometry given by id {} does not reference .+ used in "
                  "a signed distance query",
                  bad_id));

  // (3) - Invalid id_B throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.ComputeSignedDistancePairClosestPoints(id_A, bad_id, X_WGs_),
      fmt::format("The geometry given by id {} does not reference .+ used in "
                  "a signed distance query",
                  bad_id));

  // (4) - Collision filter is ignored; the pair is evaluated regardless.
  ExcludeCollisionsWithin({id_A, id_B});
  EXPECT_NO_THROW(
      engine_.ComputeSignedDistancePairClosestPoints(id_A, id_B, X_WGs_));

  // (5) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  X_WGs_ad[id_A] = RigidTransform<AutoDiffXd>{
      math::InitializeAutoDiff(X_WGs_.at(id_A).translation())};
  ad_engine->UpdateWorldPoses(X_WGs_ad);

  const SignedDistancePair<AutoDiffXd> result =
      ad_engine->ComputeSignedDistancePairClosestPoints(id_A, id_B, X_WGs_ad);
  const VectorX<double>& derivs = result.distance.derivatives();
  ASSERT_EQ(derivs.size(), 3);
  EXPECT_FALSE(derivs.isZero());
}

/* ComputeSignedDistanceToPoint() responsibilities:
  1. Report no results for an empty engine.
  2. The threshold parameter makes a difference (i.e., it is passed to the
     callback).
  3. Correct fcl formulation of query point produces expected distance
     (specifically, calls computeAABB() on the query sphere).
  4. Report distance to dynamic geometry.
  5. Report distance to anchored geometry.
  6. Results are always ordered.
  7. Collision filter doesn't matter.
  8. AutoDiff derivatives pass through successfully.
  9. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence.

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

  In this test, we create three geometries: dynamic, anchored, and dynamic to
  show the ordering of output results is independent of a geometry's dynamic
  state.
      ├┄┄┄┄┄┄ threshold ┄┄┄┄┄┄┄┄┄┄┤
                    ├┄┄┄┄┄┄ threshold ┄┄┄┄┄┄┄┄┄┄┤
            ···           ***           ···
          ·     ·       *     *       ·     ·
    Q1⏺  ·   D1  ·     *   A2  *     ·   D3  ·   ⏺Q2
         ·       ·     *       *     ·       ·
          ·     ·       *     *       ·     ·
            ···           ***           ···

  We'll query from two points, Q1 and Q2. Not all spheres are within `threshold`
  units of the query points, so the results will be limited to those within
  reach. */
TEST_F(ProximityEngineTests, ComputeSignedDistanceToPoint) {
  // Distance between sphere centers (and query points).
  const double kOffset = 1.0;
  const double kThreshold = kOffset * 2.5;
  auto eval_dut = [this, kThreshold](const Vector3d& p_WQ) {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputeSignedDistanceToPoint(p_WQ, X_WGs_, kThreshold);
  };

  // Extract the ordered ids from results.
  auto extract_ids = [](const vector<SignedDistanceToPoint<double>>& results) {
    auto ids =
        results | std::views::transform(&SignedDistanceToPoint<double>::id_G);
    return vector<GeometryId>(ids.begin(), ids.end());
  };

  // (1) - empty engine produces no results - arbitrary query point.
  EXPECT_TRUE(eval_dut({0, 0, 0}).empty());

  const Sphere sphere(0.5);
  double x = 0.0;
  auto next_x = [&x, kOffset]() {
    x += kOffset;
    return x;
  };

  const Vector3d p_WQ1(next_x(), 0, 0);
  const GeometryId dynamic1 = AddDynamic(sphere, V3{next_x(), 0, 0});
  const GeometryId anchored2 = AddAnchored(sphere, V3{next_x(), 0, 0});
  const GeometryId dynamic3 = AddDynamic(sphere, V3{next_x(), 0, 0});
  const Vector3d p_WQ2(next_x(), 0, 0);

  // (2) - (5) - we'll get distance to anchored and dynamic (but not all the
  // dynamic, based on the query point).
  // Expectations for which ids and in which order, based on query point.
  const vector<GeometryId> expected1 = {dynamic1, anchored2};
  const vector<GeometryId> expected2 = {anchored2, dynamic3};
  {
    const vector<SignedDistanceToPoint<double>> results = eval_dut(p_WQ1);
    EXPECT_EQ(extract_ids(results), expected1);
    // (3) - Q is not at (0, 0, 0), so wrong distance will reveal a bug.
    EXPECT_EQ(results[0].distance, kOffset - sphere.radius());
    EXPECT_EQ(results[1].distance, 2 * kOffset - sphere.radius());
  }
  EXPECT_EQ(extract_ids(eval_dut(p_WQ2)), expected2);

  // (6) - results are always ordered the same; repeat query with Q1.
  EXPECT_EQ(extract_ids(eval_dut(p_WQ1)), expected1);

  // (7) - Collision filters are ignored.
  ExcludeCollisionsWithin({dynamic1, anchored2});
  EXPECT_EQ(extract_ids(eval_dut(p_WQ1)), expected1);
  EXPECT_EQ(extract_ids(eval_dut(p_WQ2)), expected2);

  // (8) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Make a query point with derivatives.
  const Vector3<AutoDiffXd> p_WQ1_ad = math::InitializeAutoDiff(p_WQ1);

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  const auto ad_results =
      ad_engine->ComputeSignedDistanceToPoint(p_WQ1_ad, X_WGs_ad, kThreshold);
  // Still get two results.
  ASSERT_EQ(ad_results.size(), 2);
  for (int i = 0; i < std::ssize(ad_results); ++i) {
    // Confirm derivatives survived -- not empty and not all zero. True for both
    // dynamic and anchored geometries.
    const auto& result = ad_results[i];
    const auto& derivs = result.distance.derivatives();
    ASSERT_EQ(derivs.size(), 3);
    EXPECT_FALSE(derivs.isZero());
  }
}

/* ComputeSignedDistanceGeometryToPoint() responsibilities:
   1. Report no results for an empty engine.
   2. Reports no results for an empty geometry list.
   3. Throws for invalid geometry ids.
   4. Correct fcl formulation of query point produces expected distance
      (specifically, calls computeAABB() on the query sphere).
   5. Report distance to dynamic geometry.
   6. Report distance to anchored geometry.
   7. Results are always ordered.
   8. Collision filter doesn't matter.
   9. AutoDiff derivatives pass through successfully.
  10. It should also allow Callback exceptions to propagate through; we won't
      test this directly -- Drake standard practices and the clear absence of a
      catch block is sufficient evidence.

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

  In this test, we create three geometries: dynamic, anchored, and dynamic to
  show the ordering of output results is independent of a geometry's dynamic
  state.
            ···           ***           ···
          ·     ·       *     *       ·     ·
    Q⏺   ·   D1  ·     *   A2  *     ·   D3  ·
         ·       ·     *       *     ·       ·
          ·     ·       *     *       ·     ·
            ···           ***           ···
   */
TEST_F(ProximityEngineTests, ComputeSignedDistanceGeometryToPoint) {
  auto eval_dut = [this](const Vector3d& p_WQ,
                         const std::unordered_set<GeometryId>& ids) {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputeSignedDistanceGeometryToPoint(p_WQ, X_WGs_, ids);
  };

  // Extract the ordered ids from results.
  auto extract_ids = [](const vector<SignedDistanceToPoint<double>>& results) {
    auto ids =
        results | std::views::transform(&SignedDistanceToPoint<double>::id_G);
    return vector<GeometryId>(ids.begin(), ids.end());
  };

  // (1) - empty engine produces no results - arbitrary query point.
  EXPECT_TRUE(eval_dut({0, 0, 0}, {}).empty());

  const Sphere sphere(0.5);
  // The x-positions of all the relevant test quantities.
  const double kQx = 1.0;
  const double kD1x = 2.0;
  const double kA2x = 3.0;
  const double kD3x = 4.0;

  // This must be a non-zero point to confirm the query sphere is configured
  // properly.
  const Vector3d p_WQ(kQx, 0.0, 0.0);
  const GeometryId dynamic1 = AddDynamic(sphere, V3{kD1x, 0.0, 0.0});
  const GeometryId anchored2 = AddAnchored(sphere, V3{kA2x, 0.0, 0.0});
  const GeometryId dynamic3 = AddDynamic(sphere, V3{kD3x, 0.0, 0.0});

  // (2) - Specify no geometries, get no results.
  EXPECT_TRUE(eval_dut(p_WQ, {}).empty());

  // (3) - Invalid id throws.
  EXPECT_THROW(eval_dut(p_WQ, {GeometryId::get_new_id()}), std::exception);

  // (4)-(6) - correct distances to dynamic and anchored.
  {
    const vector<SignedDistanceToPoint<double>> results =
        eval_dut(p_WQ, {dynamic1, anchored2});
    // Compare against expected vector capturing content and order.
    EXPECT_EQ(extract_ids(results), vector<GeometryId>({dynamic1, anchored2}));
    EXPECT_EQ(results[0].distance, (kD1x - kQx) - sphere.radius());
    EXPECT_EQ(results[1].distance, (kA2x - kQx) - sphere.radius());
  }

  // (7) - Well ordered.
  EXPECT_EQ(extract_ids(eval_dut(p_WQ, {anchored2, dynamic3})),
            extract_ids(eval_dut(p_WQ, {dynamic3, anchored2})));

  // (8) - Collision filters don't matter.
  ExcludeCollisionsWithin({dynamic1, anchored2, dynamic3});
  EXPECT_EQ(eval_dut(p_WQ, {anchored2, dynamic3}).size(), 2);

  // (9) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Make a query point with derivatives.
  const Vector3<AutoDiffXd> p_WQ_ad = math::InitializeAutoDiff(p_WQ);

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  const auto ad_results = ad_engine->ComputeSignedDistanceGeometryToPoint(
      p_WQ_ad, X_WGs_ad, {dynamic1, anchored2});
  // Still get two results.
  ASSERT_EQ(ad_results.size(), 2);
  for (int i = 0; i < std::ssize(ad_results); ++i) {
    // Confirm derivatives survived -- not empty and not all zero. True for both
    // dynamic and anchored geometries.
    const auto& result = ad_results[i];
    const auto& derivs = result.distance.derivatives();
    ASSERT_EQ(derivs.size(), 3);
    EXPECT_FALSE(derivs.isZero());
  }
}

/* ComputePointPairPenetration() responsibilities:
  1. Report results for colliding dynamic-dynamic geometry pairs.
  2. Report *no* results for colliding anchored-anchored geometry pairs.
  3. Report results for colliding anchored-dynamic geometry pairs.
  4. Respect collision filter.
  5. Results are always ordered.
  6. AutoDiff derivatives pass through successfully.
  7. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence.

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion. */
TEST_F(ProximityEngineTests, ComputePointPairPenetration) {
  auto eval_dut = [this]() {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputePointPairPenetration(X_WGs_);
  };

  // Empty engines happily produce no results.
  EXPECT_TRUE(eval_dut().empty());

  const Sphere sphere{0.5};
  // Two spheres with centers 0.9 apart collide (2 * 0.5 = 1.0 > 0.9).
  const double d = 0.9;

  // Each cluster is placed far apart to prevent cross-cluster collisions.

  // Responsibility 1: colliding dynamic-dynamic pair IS reported.
  const GeometryId id_A = AddDynamic(sphere, V3{0, 0, 0});
  const GeometryId id_B = AddDynamic(sphere, V3{d, 0, 0});

  // Responsibility 2: colliding anchored-anchored pair is NOT reported.
  AddAnchored(sphere, V3{0, 100, 0});
  AddAnchored(sphere, V3{d, 100, 0});

  // Responsibility 3: colliding dynamic-anchored pair IS reported.
  const GeometryId id_C = AddDynamic(sphere, V3{0, 200, 0});
  const GeometryId id_D = AddAnchored(sphere, V3{d, 200, 0});

  // Responsibility 4: filtered dynamic-dynamic pair is NOT reported.
  const GeometryId id_G = AddDynamic(sphere, V3{0, 300, 0});
  const GeometryId id_H = AddDynamic(sphere, V3{d, 300, 0});
  ExcludeCollisionsWithin({id_G, id_H});

  const std::vector<PenetrationAsPointPair<double>> results = eval_dut();

  // Helper to extract the sorted id pairs from a result.
  auto ids_of = [](const PenetrationAsPointPair<double>& p) {
    return std::make_pair(p.id_A, p.id_B);
  };

  // (1) & (3) each contribute one pair; (2) and (4) contribute none.
  ASSERT_EQ(results.size(), 2);
  // We know that ids order by creation order; we'll exploit that.
  EXPECT_EQ(ids_of(results[0]), std::pair(id_A, id_B));
  EXPECT_EQ(ids_of(results[1]), std::pair(id_C, id_D));

  // (5) - results are stable (ordered) across repeated calls.
  const std::vector<PenetrationAsPointPair<double>> results2 = eval_dut();
  ASSERT_EQ(results2.size(), 2);
  EXPECT_EQ(ids_of(results2[0]), ids_of(results[0]));
  EXPECT_EQ(ids_of(results2[1]), ids_of(results[1]));

  // (6) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Seed id_C's translation with derivatives.
  X_WGs_ad[id_C] = RigidTransform<AutoDiffXd>{
      math::InitializeAutoDiff(X_WGs_.at(id_C).translation())};

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  const std::vector<PenetrationAsPointPair<AutoDiffXd>> ad_results =
      ad_engine->ComputePointPairPenetration(X_WGs_ad);
  // Still get two results.
  ASSERT_EQ(ad_results.size(), 2);
  // Confirm derivatives on the second result -- not empty and not zero.
  const auto& result = ad_results[1];
  const auto& derivs = result.depth.derivatives();
  ASSERT_EQ(derivs.size(), 3);
  EXPECT_FALSE(derivs.isZero());
}

// Even when a geometry is inactive, it should still be included in distance to
// point queries.
TEST_F(ProximityEngineTests, InactiveGeometryInDistanceToPoint) {
  const Sphere sphere(0.5);
  const GeometryId id_A = AddDynamic(sphere, V3{0, 0, 0});

  {
    const std::vector<SignedDistanceToPoint<double>> results =
        engine_.ComputeSignedDistanceToPoint(V3{0, 0, 1}, X_WGs_);
    ASSERT_EQ(results.size(), 1);
    ASSERT_EQ(results[0].id_G, id_A);
  }

  Deactivate({id_A});
  ASSERT_TRUE(engine_.IsInactiveDynamic(id_A));

  {
    const std::vector<SignedDistanceToPoint<double>> results =
        engine_.ComputeSignedDistanceToPoint(V3{0, 0, 1}, X_WGs_);
    ASSERT_EQ(results.size(), 1);
    ASSERT_EQ(results[0].id_G, id_A);
  }
}

// ProximityEngine uses a "staleness" classification of inactive geometry to
// defer expensive calculations. This test covers the operations that leave the
// inactive set in a *non*-stale state. That includes a small set:
//
//  a. Original construction (the inactive set is empty).
//  b. After calling ComputeSignedDistanceToPoint().
TEST_F(ProximityEngineTests, InactiveGeometryNotStale) {
  const Box shape(0.5, 0.5, 0.5);
  const GeometryId id_A = AddDynamic(shape);

  // (a) Upon construction, the engine is not stale.
  ASSERT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  // While not part of this test, we'll make the geometry inactive and use the
  // side effect of UpdateWorldPoses() to make the inactive set stale (tested
  // below).
  Deactivate({id_A});
  engine_.UpdateWorldPoses(X_WGs_);
  EXPECT_TRUE(Tester::is_inactive_dynamic_stale(engine_));

  // (b) Calling ComputeSignedDistanceToPoint() clears the staleness.
  engine_.ComputeSignedDistanceToPoint(V3{0, 0, 1}, X_WGs_);
  EXPECT_FALSE(Tester::is_inactive_dynamic_stale(engine_));
}

// ProximityEngine uses a "staleness" classification of inactive geometry to
// defer expensive calculations. This test covers the operations that leave the
// inactive set in a *stale* state. That includes the following:
//
//  a. Copy construction - the copy is stale, but the source is unchanged.
//  b. Scalar-converted copies also start stale.
//  c. Calling UpdateWorldPoses() when there are inactive geometries.
//  d. Updating margins on hydro-compliant, inactive geometry.
TEST_F(ProximityEngineTests, InactiveGeometryMakeStale) {
  const Box shape(0.5, 0.5, 0.5);
  const GeometryId id_A = AddDynamic(shape);

  // We'll use the side effect of ComputeSignedDistanceToPoint() to clear
  // staleness as pre-conditions for the various tests (tested above).
  auto clear_staleness = [this](const auto& engine) {
    engine.ComputeSignedDistanceToPoint(V3{0, 0, 1}, X_WGs_);
  };

  // Original is not stale.
  ASSERT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  {
    // (a) Copy construction produces stale copy.
    ProximityEngine<double> copied(engine_);
    EXPECT_TRUE(Tester::is_inactive_dynamic_stale(copied));

    // (b) Scalar-converted copy is also stale.
    const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
    EXPECT_TRUE(Tester::is_inactive_dynamic_stale(*ad_engine));
  }

  // Neither operation changed the original; still not stale.
  ASSERT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  // (c) without inactive geometry, UpdateWorldPoses() does *not* make it stale.
  engine_.UpdateWorldPoses(X_WGs_);
  EXPECT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  // (c) *with* inactive geometry,  UpdateWorldPoses() makes it stale.
  Deactivate({id_A});
  engine_.UpdateWorldPoses(X_WGs_);
  EXPECT_TRUE(Tester::is_inactive_dynamic_stale(engine_));

  // (d) Updating new properties on *inactive* geometries.
  {
    ASSERT_TRUE(engine_.IsInactiveDynamic(id_A));

    InternalGeometry geo(SourceId::get_new_id(), std::make_unique<Box>(shape),
                         FrameId::get_new_id(), id_A, "sphere",
                         RigidTransformd());
    ProximityProperties props;
    AddCompliantHydroelasticProperties(1.0, 1e17, &props);

    clear_staleness(engine_);
    ASSERT_FALSE(Tester::is_inactive_dynamic_stale(engine_));
    engine_.UpdateRepresentationForNewProperties(geo, props);
    EXPECT_TRUE(Tester::is_inactive_dynamic_stale(engine_));
  }
}

// ProximityEngine uses a "staleness" classification of inactive geometry to
// defer expensive calculations. This test covers the operations whose behavior
// is determined by the staleness -- specifically which objects get updated
// when:
//
//   a. Deactivate() does not change staleness.
//      i. But, if *not* stale, the deactivated geometry will get its AABB
//         updated.
//   b. Activate() does not change staleness.
//      i. But, if stale, the activated geometry will get its AABB updated.
//   c. UpdateWorldPoses() never updates the AABBs of inactive geometries
//      (but it *does* change the poses).
//   d. Multiple calls to ComputeSignedDistanceToPoint() (without any
//      intervening staleness triggers) will only update the inactive
//      geometry poses on the first call.
TEST_F(ProximityEngineTests, InactiveGeometryStaleSemantics) {
  const Box shape(0.5, 0.5, 0.5);
  const GeometryId id_A = AddDynamic(shape);

  // We'll use the side effect of ComputeSignedDistanceToPoint() to clear
  // staleness as pre-conditions for the various tests.
  auto clear_staleness = [this](const auto& engine) {
    engine.ComputeSignedDistanceToPoint(V3{0, 0, 1}, X_WGs_);
  };

  // (a) Deactivate does *not* make it stale.
  Deactivate({id_A});
  EXPECT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  // (b) Activate does *not* make it stale.
  Activate({id_A});
  EXPECT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  // We need to test the effect of the various functions on the poses and the
  // AABBs of geometries. We *know* that each collision geometry's AABB is
  // expressed in the world frame. So, simply by moving the geometry, we should
  // observe in the min & max coordinates of the AABB (we'll use min coordinates
  // as evidence of change).
  //
  // set_pose - a function to update the pose of the geometry. It should *never*
  //            update the AABB of an inactive geometry (explicitly tested).
  // dut - the operation under test. It may or may not update the AABB based on
  //       our expectations (expect_aabb_change).
  auto validate_expected_change =
      [this](const GeometryId id, bool expect_aabb_change,
             std::function<void(GeometryId, const Vector3d& p_WO)> set_pose,
             std::function<void()> dut, const std::string& description) {
        SCOPED_TRACE(description);
        const fcl::CollisionObjectd* obj =
            Tester::GetCollisionObject(engine_, id);
        ASSERT_NE(obj, nullptr);
        // Start with translation and AABB in agreement.
        const_cast<fcl::CollisionObjectd*>(obj)->computeAABB();
        const Vector3d p_WO_before = obj->getTranslation();
        const Vector3d min_before = obj->getAABB().min_;

        // Guarantee the new pose is different from the old.
        const Vector3d p_WO_target(p_WO_before + Vector3d(1, 2, 3));
        set_pose(id, p_WO_target);

        // set_pose() should never update an inactive AABB.
        if (engine_.IsInactiveDynamic(id)) {
          EXPECT_TRUE(CompareMatrices(obj->getAABB().min_, min_before));
        }

        dut();

        const Vector3d p_WO_after = obj->getTranslation();
        const Vector3d min_after = obj->getAABB().min_;

        // Pose should always be updated.
        EXPECT_TRUE(CompareMatrices(p_WO_after, p_WO_target));
        EXPECT_EQ(CompareMatrices(min_before, min_after), !expect_aabb_change);
      };

  // Updates the pose of the geometry indicated by id in such a way that the
  // engine will become stale -- uses UpdateWorldPoses().
  auto update_pose_and_make_stale = [this](const GeometryId id,
                                           const Vector3d& p_WQ) {
    X_WGs_[id] = RigidTransformd(p_WQ);
    engine_.UpdateWorldPoses(X_WGs_);
    EXPECT_TRUE(Tester::is_inactive_dynamic_stale(engine_));
  };

  // Updates the pose of the geometry indicated by id by setting it under the
  // hood, preventing the staleness of the inactive set from changing.
  auto update_pose = [this](const GeometryId id, const Vector3d& p_WQ) {
    const bool pre_staleness = Tester::is_inactive_dynamic_stale(engine_);
    const fcl::CollisionObjectd* obj = Tester::GetCollisionObject(engine_, id);
    ASSERT_NE(obj, nullptr);
    // Start with translation and AABB in agreement.
    const_cast<fcl::CollisionObjectd*>(obj)->setTranslation(p_WQ);
    // We did not change staleness.
    EXPECT_EQ(pre_staleness, Tester::is_inactive_dynamic_stale(engine_));
  };

  // (a.i) Deactivate() into a non-stale inactive set updates the AABB.
  Activate({id_A});
  ASSERT_FALSE(engine_.IsInactiveDynamic(id_A));
  clear_staleness(engine_);
  ASSERT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  validate_expected_change(
      id_A,
      /* expect_aabb_change= */ true, update_pose,
      [this, id_A]() {
        Deactivate({id_A});
      },
      "Deactivate() updates non-stale AABB");

  // (b.i) Activate() from a stale state updates the AABB of the activated
  // geometry when the inactive tree is stale.
  ASSERT_TRUE(engine_.IsInactiveDynamic(id_A));

  validate_expected_change(
      id_A,
      /* expect_aabb_change= */ true, update_pose_and_make_stale,
      [this, id_A]() {
        Activate({id_A});
      },
      "Activate() updates stale AABB");

  // (c) UpdateWorldPoses() updates the inactive geometry's pose but not its
  // AABB.
  Deactivate({id_A});
  ASSERT_TRUE(engine_.IsInactiveDynamic(id_A));
  validate_expected_change(
      id_A,
      /* expect_aabb_change= */ false,
      update_pose_and_make_stale,  // This calls UpdateWorldPoses() internally.
      []() {},  // Do nothing; just evaluating the effect of UpdateWorldPoses().
      "UpdateWorldPoses() updates inactive poses, but not its AABB");

  // (d) For multiple calls to ComputeSignedDistanceToPoint() only the first
  // updates the poses on inactive geometries.
  ASSERT_TRUE(engine_.IsInactiveDynamic(id_A));

  auto eval_distance = [this]() {
    engine_.ComputeSignedDistanceToPoint(V3{0, 0, 1}, X_WGs_);
  };

  // The first time, the we'll make it stale by updating the pose and we'll
  // see the pose and AABB updated by evaluating distance.
  validate_expected_change(
      id_A,
      /* expect_aabb_change= */ true, update_pose_and_make_stale, eval_distance,
      "ComputeSignedDistanceToPoint() updates stale inactive set");

  ASSERT_FALSE(Tester::is_inactive_dynamic_stale(engine_));

  // This time we update the pose via the back door, the next distance
  // evaluation will not change the AABB.
  validate_expected_change(
      id_A,
      /* expect_aabb_change= */ false, update_pose, eval_distance,
      "ComputeSignedDistanceToPoint() does not update non-stale inactive set");
}

/* When geometries are (de)activated, ProximityEngine's callback reclassifies
 the geometry as active/inactive in response.

  1. Marking an active geometry as inactive, moves it out of dynamic geometry
     and into inactive.
  2. Marking an _inactive_ geometry as inactive has no effect.
  3. Marking an inactive geometry as active, moves it back to dynamic geometry.
  4. Marking an active geometry as active has no effect.

 Note: this doesn't address the handling of inactive "staleness" -- see the
 InactiveGeometryStaleness test for that. */
TEST_F(ProximityEngineTests, ApplyActiveStatusChange) {
  const Sphere sphere{0.5};
  const GeometryId id_A = AddDynamic(sphere, V3{0, 0, 0});
  const GeometryId id_B = AddDynamic(sphere, V3{1, 0, 0});
  EXPECT_EQ(engine_.num_dynamic(), 2);
  EXPECT_EQ(engine_.num_inactive_dynamic(), 0);

  // (1) - Marking an active geometry as inactive moves it to inactive.
  Deactivate({id_A});
  EXPECT_TRUE(engine_.IsInactiveDynamic(id_A));
  EXPECT_FALSE(engine_.IsInactiveDynamic(id_B));
  EXPECT_EQ(engine_.num_dynamic(), 2);
  EXPECT_EQ(engine_.num_inactive_dynamic(), 1);

  // (2) - Marking an inactive geometry as inactive has no effect.
  Deactivate({id_A});
  EXPECT_TRUE(engine_.IsInactiveDynamic(id_A));
  EXPECT_FALSE(engine_.IsInactiveDynamic(id_B));
  EXPECT_EQ(engine_.num_dynamic(), 2);
  EXPECT_EQ(engine_.num_inactive_dynamic(), 1);

  // (3) - Marking an inactive geometry as active moves it back to dynamic.
  Activate({id_A});
  EXPECT_FALSE(engine_.IsInactiveDynamic(id_A));
  EXPECT_FALSE(engine_.IsInactiveDynamic(id_B));
  EXPECT_EQ(engine_.num_dynamic(), 2);
  EXPECT_EQ(engine_.num_inactive_dynamic(), 0);

  // (4) - Marking an active geometry as active has no effect.
  Activate({id_A});
  EXPECT_FALSE(engine_.IsInactiveDynamic(id_A));
  EXPECT_FALSE(engine_.IsInactiveDynamic(id_B));
  EXPECT_EQ(engine_.num_dynamic(), 2);
  EXPECT_EQ(engine_.num_inactive_dynamic(), 0);
}

/* FindCollisionCandidates() responsibilities:
  1. Report no candidates for an empty engine.
  2. Do not report anchored-anchored pairs, even if their AABBs overlap.
  3. Report dynamic-dynamic pairs whose AABBs overlap.
  4. Respect collision filter (filtered pairs are not candidates).
  5. Report dynamic-anchored pairs whose AABBs overlap.
  6. Results are stable (ordered) across repeated calls.
  7. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence.

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

  We set up several spheres. The letter indicates anchored (A) or dynamic (D);
  the subscript indicates registration order.

  We set up spheres as follows:

       ···           ***
     ·     ·       *     *     - A1, A2 anchored spheres that overlap each
    ·   A1  ·     *   D3  *      other.
    ·       ·     *       *    - D3, D4 dynamic spheres that overlap each other.
     · ··· ·       * *** *     - D5 a *massive* dynamic sphere (boundaries not
       ···    D5     ***         shown) that overlaps all the other geometries.
     ·     ·       *     *
    ·       ·     *       *    - Prefixes A, D indicate anchored or dynamic.
    ·   A2  ·     *   D4  *    - Suffix numbers indicate order created.
     ·     ·       *     *
       ···           ***
*/
TEST_F(ProximityEngineTests, FindCollisionCandidates) {
  auto eval_dut = [this]() {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.FindCollisionCandidates();
  };
  const Sphere sphere{0.5};
  // Distance less than two radii implies collision.
  const double d = 2 * sphere.radius() - 0.1;

  // (1) - empty engine has no candidates.
  ASSERT_TRUE(eval_dut().empty());

  // (2) - anchored-anchored pairs are never candidates.
  const GeometryId anchored1 = AddAnchored(sphere);
  const GeometryId anchored2 = AddAnchored(sphere, V3{0, -d, 0});
  ASSERT_TRUE(eval_dut().empty());

  // (3) - dynamic-dynamic pairs whose AABBs overlap ARE candidates.
  const GeometryId dynamic3 = AddDynamic(sphere, V3{2, 0, 0});
  const GeometryId dynamic4 = AddDynamic(sphere, V3{2, -d, 0});
  ASSERT_THAT(eval_dut(), ::testing::UnorderedElementsAre(
                              SortedPair<GeometryId>(dynamic3, dynamic4)));

  // (4) - a filtered pair is not a candidate.
  std::optional<FilterId> filter_id =
      ExcludeCollisionsWithin({dynamic3, dynamic4}, /* is_temporary= */ true);
  DRAKE_DEMAND(filter_id.has_value());
  ASSERT_TRUE(eval_dut().empty());
  engine_.collision_filter().RemoveDeclaration(*filter_id);

  // (5) - dynamic-anchored pairs whose AABBs overlap ARE candidates.
  const GeometryId dynamic5 = AddDynamic(Sphere(5));
  const vector<SortedPair<GeometryId>> results = eval_dut();
  // This is the expected *contents* but not the order of the results.
  const std::set<SortedPair<GeometryId>> expected{
      {dynamic3, dynamic4}, {dynamic5, anchored1}, {dynamic5, anchored2},
      {dynamic5, dynamic3}, {dynamic5, dynamic4},
  };
  ASSERT_THAT(eval_dut(), ::testing::UnorderedElementsAreArray(expected));

  // (6) - the five spheres we have in the engine are enough to expose FCL
  // ordering instability. A second invocation will be sufficient to show
  // ordering consistency despite that instability.
  const vector<SortedPair<GeometryId>> results2 = eval_dut();
  EXPECT_EQ(results, results2);
}

/* HasCollisions() responsibilities:

  1. Report false for an empty engine.
  2. Report false when only anchored-anchored geometry pairs collide.
  3. Report true for colliding dynamic-dynamic geometry pairs.
  4. Respect collision filter (filtered pair does not count as a collision).
  5. Report true for colliding anchored-dynamic geometry pairs.
  6. Non-colliding anchored geometry with bounding-box overlap does not suppress
     dynamic-dynamic collision reporting.

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

  We'll set up a number spheres. The letter indicates whether it is anchored or
  dynamic. The index indicates the order in which the sphere is added.
      ···      ***
    ·     ·  *     *
  ·        *·        *
  ·   D3  ▫▫▫  D4    *   -- Dynamic spheres D3 and D4 intersect each other
    ·   ▫ ·  *▫    *        but not anchored geometries
      ▫··      *▫*
      ▫   D5₂   ▫        -- D5₂ intersects with D3 & D4, but *not* A1 and A2.
        ▫     ▫             But D5₂'s BV will overlap with A1's and A2's BVs.
      ··· ▫▫▫  ***
    ·     ·  *     *
  ·        *·        *   -- Anchored spheres A1 and A2 intersect each other.
  ·   A1   *·  A2    *
    · ▴▴▴ ·  *     *
    ▴ ··· ▴    ***
  ▴         ▴
  ▴   D5₁   ▴            -- D5₁ intersects with A1.
    ▴     ▴
      ▴▴▴

  D5 has two different poses: D5₁ and D5₂. */
TEST_F(ProximityEngineTests, HasCollisions) {
  auto eval_dut = [this]() {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.HasCollisions();
  };

  const Sphere sphere{0.5};
  // Distance less than two radii implies collision.
  const double d = 2 * sphere.radius() - 0.1;

  // (1) - empty engine reports no collisions.
  ASSERT_FALSE(eval_dut());

  // (2) - anchored-anchored collisions are not reported.
  const GeometryId anchored1 = AddAnchored(sphere);
  // We don't need to know A2's id, so we'll leave it anonymous.
  AddAnchored(sphere, Vector3d{d, 0, 0});
  ASSERT_FALSE(eval_dut());

  // (3) - dynamic-dynamic are reported -- note: these are separated from the
  // anchored geometries above -- not even their AABBs overlap.
  const GeometryId dynamic3 = AddDynamic(sphere, V3{0, 1.5 * d, 0});
  const GeometryId dynamic4 = AddDynamic(sphere, V3{d, 1.5 * d, 0});

  ASSERT_TRUE(eval_dut());

  // (4) - Filtering the dynamic pair, removes the only collision.
  ExcludeCollisionsWithin({dynamic3, dynamic4});
  ASSERT_FALSE(eval_dut());

  // (5) - dynamic-anchored are reported.
  // Place D5 in pose D5₁.
  const GeometryId dynamic5 = AddDynamic(sphere, V3{0, -d, 0});
  ASSERT_TRUE(eval_dut());

  // (6) - Weird special case. There was a legacy bug (reported in issue #23406)
  // where collisions between dynamic geometries were erased if there were
  // no anchored-dynamic collisions, but there were overlapping bounding volumes
  // between anchored and dynamic geometries. So, we move D5 to position D5₂
  X_WGs_[dynamic5] = RigidTransformd(V3{d / 2, d, 0});

  // First confirm that D5₂ satisfies both requirements:
  //   1. It doesn't collide with the anchored geometry.
  //      - temporarily filter collisions between dynamic geometries and confirm
  //        no collisions reported.
  //   2. Its AABB overlaps with those of the anchored geometry.
  //      Confirm that, even with the dynamic filter in place, we are still
  //      getting collision candidates (including D5 but not D3 or D4).
  //      We'll assume FindCollisionCandidates() has tests proving it a reliable
  //      witness.
  std::optional<FilterId> filter_id = ExcludeCollisionsWithin(
      {dynamic3, dynamic4, dynamic5}, /* is_temporary= */ true);
  DRAKE_DEMAND(filter_id.has_value());
  ASSERT_FALSE(eval_dut());
  const vector<SortedPair<GeometryId>> candidates =
      engine_.FindCollisionCandidates();
  ASSERT_GT(candidates.size(), 0);
  ASSERT_THAT(candidates,
              ::testing::Contains(SortedPair<GeometryId>(dynamic5, anchored1)));

  // Remove the filter and confirm we get collisions.
  engine_.collision_filter().RemoveDeclaration(*filter_id);
  ASSERT_TRUE(eval_dut());
}

// When anchored geometry is added to the proximity engine, the broadphase
// algorithm needs to be properly updated, otherwise it assumes all of the
// anchored geometry has the identity transformation. This test confirms that
// this configuration occurs; the dynamic sphere and anchored sphere are
// configured away from the origin in collision. Without proper broadphase
// initialization for the anchored geometry, no collision is reported. We
// also confirm that it gets initialized properly when cloned.
//
// This test uses HasCollisions() to exercise the broadphase, but it is not
// a test on HasCollisions() logic (that is tested elsewhere).
TEST_F(ProximityEngineTests, AnchoredBroadPhaseInitialization) {
  const Sphere sphere(0.5);

  AddDynamic(sphere, V3{-3, 0.75, 0});
  AddAnchored(sphere, V3{-3, 0, 0});

  engine_.UpdateWorldPoses(X_WGs_);

  EXPECT_TRUE(engine_.HasCollisions());

  // Confirm that it survives copying.
  ProximityEngine<double> engine_copy(engine_);
  engine_copy.UpdateWorldPoses(X_WGs_);
  EXPECT_TRUE(engine_copy.HasCollisions());
}

TEST_F(ProximityEngineTests, NeedsConvexHull) {
  const SourceId s_id = SourceId::get_new_id();
  const FrameId f_id = FrameId::get_new_id();
  const GeometryId g_id = GeometryId::get_new_id();

  // Shapes that don't require convex hulls.
  vector<std::unique_ptr<Shape>> unsupported_shapes;
  unsupported_shapes.push_back(make_unique<Box>(1, 1, 1));
  unsupported_shapes.push_back(make_unique<Capsule>(1, 1));
  unsupported_shapes.push_back(make_unique<Cylinder>(1, 1));
  unsupported_shapes.push_back(make_unique<Ellipsoid>(1, 2, 3));
  unsupported_shapes.push_back(make_unique<HalfSpace>());
  unsupported_shapes.push_back(make_unique<MeshcatCone>(1));
  unsupported_shapes.push_back(make_unique<Sphere>(1));
  for (std::unique_ptr<Shape>& shape : unsupported_shapes) {
    InternalGeometry geo(s_id, std::move(shape), f_id, g_id, "name", {});
    EXPECT_FALSE(engine_.NeedsConvexHull(geo));
  }

  // Rigid shapes that *do* require convex hulls.
  EXPECT_TRUE(engine_.NeedsConvexHull(InternalGeometry(
      s_id, make_unique<Mesh>("unimportant"), f_id, g_id, "name", {})));
  EXPECT_TRUE(engine_.NeedsConvexHull(InternalGeometry(
      s_id, make_unique<Convex>("unimportant"), f_id, g_id, "name", {})));

  // Being deformable would eliminate the need for a convex hull. Passing
  // resolution hint declares the internal geometry as deformable.
  EXPECT_FALSE(engine_.NeedsConvexHull(InternalGeometry(
      s_id,
      make_unique<Mesh>(
          FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk")),
      f_id, g_id, "name", {}, /* resolution_hint= */ 1.0)));
}

// ProximityEngine creates fcl::Convexd for all Mesh and Convex. For Convex,
// it's part of the contract. For Mesh, it is the current handicapped
// implementation.
//
// This confirms that the input mesh is ignored in favor of the convex hull
// provided to ProximityEngine.
TEST_F(ProximityEngineTests, ImplementedAsFclConvex) {
  // This mesh has 8 small wedges jammed into the corners of a cube 2-units on
  // the side, centered on the origin. It is decidedly non-convex.
  // In collision queries, it should respond like a solid cube.
  const std::string obj_path =
      FindResourceOrThrow("drake/geometry/test/cube_corners.obj");

  auto expect_fcl_convex_is_cube = [this](GeometryId id) {
    const fcl::CollisionObjectd* object =
        ProximityEngineTester::GetCollisionObject(engine_, id);
    DRAKE_DEMAND(object != nullptr);
    ASSERT_EQ(object->getNodeType(), fcl::GEOM_CONVEX);
    const fcl::Convexd* fcl_shape =
        dynamic_cast<const fcl::Convexd*>(object->collisionGeometry().get());
    DRAKE_DEMAND(fcl_shape != nullptr);
    // If we have a cube's 8 vertices and six faces, we'll assume it's a cube.
    EXPECT_EQ(fcl_shape->getVertices().size(), 8);
    EXPECT_EQ(fcl_shape->getFaceCount(), 6);
  };

  {
    SCOPED_TRACE("Mesh as fcl::Convexd");

    const GeometryId id = AddAnchored(Mesh(obj_path));
    expect_fcl_convex_is_cube(id);
  }
  {
    SCOPED_TRACE("Convex as fcl::Convexd");
    const GeometryId id = AddAnchored(Convex(obj_path));
    expect_fcl_convex_is_cube(id);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
