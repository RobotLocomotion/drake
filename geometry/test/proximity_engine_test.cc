#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
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
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/proximity/deformable_contact_internal.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
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
    return engine.GetX_WG(id, is_dynamic);
  }

  template <typename T>
  static HydroelasticType hydroelastic_type(GeometryId id,
                                            const ProximityEngine<T>& engine) {
    return engine.hydroelastic_geometries().hydroelastic_type(id);
  }

  template <typename T>
  static bool IsFclConvexType(const ProximityEngine<T>& engine, GeometryId id) {
    return engine.IsFclConvexType(id);
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
  static const internal::deformable::Geometries&
  get_deformable_contact_geometries(const ProximityEngine<T>& engine) {
    return engine.deformable_contact_geometries();
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

namespace deformable {

class GeometriesTester {
 public:
  GeometriesTester() = delete;

  static const DeformableGeometry& get_deformable_geometry(
      const Geometries& geometries, GeometryId id) {
    return geometries.deformable_geometries_.at(id);
  }

  static const RigidGeometry& get_rigid_geometry(const Geometries& geometries,
                                                 GeometryId id) {
    return geometries.rigid_geometries_.at(id);
  }

  static void disable_rigid_geometry_deferral(Geometries* geometries) {
    geometries->enable_rigid_geometries_pending_ = false;
  }
};

}  // namespace deformable

namespace {

using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
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

// Test simple addition of dynamic geometry.
GTEST_TEST(ProximityEngineTests, AddDynamicGeometry) {
  ProximityEngine<double> engine;
  Sphere sphere{0.5};
  const GeometryId id = GeometryId::get_new_id();
  engine.AddDynamicGeometry(sphere, {}, id);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// "Processing" hydroelastic geometry is mostly a case of invoking a method
// on hydroelastic::Geometries. Assuming Geometries successfully adds a
// geometry, ProximityEngine has an additional task to inflate fcl AABB bounding
// volumes for compliant hydroelastic geometries with non-zero margins. This
// test merely confirms that meshes with hydroelastic properties invoke
// hydroelastic::Geometries (relying on its tests to cover the details). So,
// we'll simply attempt to instantiate every shape and assert its classification
// based on whether it's supported or not (note: this test doesn't depend on the
// choice of rigid/compliant -- for each shape, we pick an arbitrary compliance
// type, preferring one that is supported over one that is not. Otherwise, the
// compliance choice is immaterial.) One exception is that the rigid Mesh and
// the compliant Mesh use two different kinds of files, so we test both of them.
// The test HydroelasticAabbInflation covers ProximityEngine's second task.
GTEST_TEST(ProximityEngineTests, ProcessHydroelasticProperties) {
  ProximityEngine<double> engine;
  // All of the geometries will have a scale comparable to edge_length, so that
  // the mesh creation is as cheap as possible. The exception is Mesh
  // geometry since we have no re-meshing.
  const double edge_length = 0.5;
  const double E = 1e8;  // Elastic modulus.
  ProximityProperties soft_properties;
  AddCompliantHydroelasticProperties(edge_length, E, &soft_properties);
  ProximityProperties rigid_properties;
  AddRigidHydroelasticProperties(edge_length, &rigid_properties);

  // Case: compliant sphere.
  {
    Sphere sphere{edge_length};
    const GeometryId sphere_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(sphere, {}, sphere_id, soft_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(sphere_id, engine),
              HydroelasticType::kCompliant);
  }

  // Case: rigid cylinder.
  {
    Cylinder cylinder{edge_length, edge_length};
    const GeometryId cylinder_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(cylinder, {}, cylinder_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(cylinder_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: rigid ellipsoid.
  {
    Ellipsoid ellipsoid{edge_length, edge_length, edge_length};
    const GeometryId ellipsoid_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(ellipsoid, {}, ellipsoid_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(ellipsoid_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: rigid capsule.
  {
    Capsule capsule{edge_length, edge_length};
    const GeometryId capsule_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(capsule, {}, capsule_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(capsule_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: rigid half_space.
  {
    HalfSpace half_space;
    const GeometryId half_space_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(half_space, {}, half_space_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(half_space_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: rigid box.
  {
    Box box{edge_length, edge_length, edge_length};
    const GeometryId box_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(box, {}, box_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(box_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: rigid mesh.
  {
    Mesh mesh(
        drake::FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"));
    const GeometryId mesh_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(mesh, {}, mesh_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(mesh_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: compliant mesh.
  {
    Mesh mesh(
        drake::FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk"));
    const GeometryId mesh_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(mesh, {}, mesh_id, soft_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(mesh_id, engine),
              HydroelasticType::kCompliant);
  }

  // Case: rigid mesh vtk.
  {
    Mesh mesh(
        drake::FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk"));
    const GeometryId mesh_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(mesh, {}, mesh_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(mesh_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: compliant convex vtk.
  {
    Convex convex(
        drake::FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk"));
    const GeometryId convex_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(convex, {}, convex_id, soft_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(convex_id, engine),
              HydroelasticType::kCompliant);
  }

  // Case: rigid convex.
  {
    Convex convex{
        drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
        edge_length};
    const GeometryId convex_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(convex, {}, convex_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(convex_id, engine),
              HydroelasticType::kRigid);
  }

  // Case: rigid convex vtk.
  {
    Convex convex{
        drake::FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk"),
        edge_length};
    const GeometryId convex_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(convex, {}, convex_id, rigid_properties);
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(convex_id, engine),
              HydroelasticType::kRigid);
  }
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
GTEST_TEST(ProximityEngineTests, HydroelasticAabbInflation) {
  ProximityEngine<double> engine;
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

  ProximityProperties soft_properties(no_hydro_properties);
  AddCompliantHydroelasticProperties(edge_length, E, &soft_properties);

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
      {&box, soft_properties, Vector3d::Constant(-0.5 - kMarginValue),
       Vector3d::Constant(0.5 + kMarginValue), true,
       "Soft primitive-> exact inflation"},
      {&mesh, soft_properties, mesh_min - Vector3d::Constant(kMarginValue),
       mesh_max + Vector3d::Constant(kMarginValue), false,
       "Soft mesh -> inflation exceeds minimum"},
      {&convex, soft_properties, mesh_min - Vector3d::Constant(kMarginValue),
       mesh_max + Vector3d::Constant(kMarginValue), false,
       "Soft convex -> inflation exceeds minimum"},
      {&vol_mesh, soft_properties, vol_min - Vector3d::Constant(kMarginValue),
       vol_max + Vector3d::Constant(kMarginValue), false,
       "Soft vol mesh -> inflation exceeds minimum"},
  };

  for (const auto& test_case : cases) {
    const GeometryId id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(*test_case.shape, {}, id, test_case.properties);
    const auto* fcl = ProximityEngineTester::GetCollisionObject(engine, id);
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
GTEST_TEST(ProximityEngineTests, MarginAfterPropertyUpdate) {
  ProximityEngine<double> engine;
  // All of the geometries will have a scale comparable to edge_length, so that
  // the mesh creation is as cheap as possible. The exception is Mesh
  // geometry since we have no re-meshing.
  const double kRadius = 0.5;
  const double kResHint = kRadius / 5;
  const double kE = 1e8;            // Elastic modulus.
  const double kMarginValue = 0.1;  // Large margin value for clarity.
  const double kDistance = 2 * (kRadius + kMarginValue) - 0.5 * kMarginValue;

  const Sphere sphere(kRadius);
  const RigidTransformd X_WA(Vector3d(kDistance, 0, 0));

  ProximityProperties soft_properties;
  soft_properties.AddProperty(kHydroGroup, kMargin, 0.0);
  AddCompliantHydroelasticProperties(kResHint, kE, &soft_properties);

  // Shape A starts with zero margin.
  const GeometryId idA = GeometryId::get_new_id();
  engine.AddDynamicGeometry(sphere, {}, idA, soft_properties);

  // Shape B gets margin.
  soft_properties.UpdateProperty(kHydroGroup, kMargin, kMarginValue);
  const GeometryId idB = GeometryId::get_new_id();
  engine.AddAnchoredGeometry(sphere, {}, idB, soft_properties);

  const std::unordered_map<GeometryId, math::RigidTransformd> poses = {
      {idA, X_WA}, {idB, RigidTransformd()}};
  engine.UpdateWorldPoses(poses);

  // idA has no margin; there should be no collision candidates.
  EXPECT_EQ(engine.FindCollisionCandidates().size(), 0);

  // Now update A to get the margin. (To do so, we need a dummy InternalGeometry
  // associated with the id).
  const InternalGeometry geo_A(SourceId::get_new_id(),
                               std::make_unique<Sphere>(kRadius),
                               FrameId::get_new_id(), idA, "A", X_WA);
  engine.UpdateRepresentationForNewProperties(geo_A, soft_properties);

  // Now they overlap and should be reported as a collision candidate.
  EXPECT_EQ(engine.FindCollisionCandidates().size(), 1);
}

// The same test set up as MarginAfterPropertyUpdate. However, this time we
// confirm that margins survive cloning.
GTEST_TEST(ProximityEngineTetsts, MarginAfterEngineClone) {
  ProximityEngine<double> engine;
  // All of the geometries will have a scale comparable to edge_length, so that
  // the mesh creation is as cheap as possible. The exception is Mesh
  // geometry since we have no re-meshing.
  const double kRadius = 0.5;
  const double kResHint = kRadius / 5;
  const double kE = 1e8;            // Elastic modulus.
  const double kMarginValue = 0.1;  // Large margin value for clarity.
  const double kDistance = 2 * (kRadius + kMarginValue) - 0.5 * kMarginValue;

  const Sphere sphere(kRadius);
  const RigidTransformd X_WA(Vector3d(kDistance, 0, 0));

  ProximityProperties soft_properties;
  soft_properties.AddProperty(kHydroGroup, kMargin, kMarginValue);
  AddCompliantHydroelasticProperties(kResHint, kE, &soft_properties);

  // Both shapes have margin.
  const GeometryId idA = GeometryId::get_new_id();
  engine.AddDynamicGeometry(sphere, {}, idA, soft_properties);
  const GeometryId idB = GeometryId::get_new_id();
  engine.AddAnchoredGeometry(sphere, {}, idB, soft_properties);

  const std::unordered_map<GeometryId, math::RigidTransformd> poses = {
      {idA, X_WA}, {idB, RigidTransformd()}};
  engine.UpdateWorldPoses(poses);

  // The original engine reports contact.
  EXPECT_EQ(engine.FindCollisionCandidates().size(), 1);

  // A copy reports the same.
  ProximityEngine<double> copy(engine);
  EXPECT_EQ(copy.FindCollisionCandidates().size(), 1);
}

// Test a combination that used to throw an exception.
GTEST_TEST(ProximityEngineTests, ProcessVtkMeshUndefHydro) {
  ProximityEngine<double> engine;

  // Case: mesh vtk, no hydro type annotation.
  {
    Mesh mesh(
        drake::FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk"));
    const GeometryId mesh_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(mesh, {}, mesh_id, ProximityProperties());
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(mesh_id, engine),
              HydroelasticType::kUndefined);
  }
}

// Test a combination that used to throw an exception.
GTEST_TEST(ProximityEngineTests, ProcessVtkConvexUndefHydro) {
  ProximityEngine<double> engine;

  // Case: convex vtk, no hydro type annotation.
  {
    Convex convex(
        drake::FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk"));
    const GeometryId convex_id = GeometryId::get_new_id();
    engine.AddDynamicGeometry(convex, {}, convex_id, ProximityProperties());
    EXPECT_EQ(ProximityEngineTester::hydroelastic_type(convex_id, engine),
              HydroelasticType::kUndefined);
  }
}

// Tests that registering the same mesh source multiple times reuses the same
// underlying fcl::Convex collision geometry object (i.e. the convex hull cache
// is working).
GTEST_TEST(ProximityEngineTests, ConvexHullCacheDuplicatesShareGeometry) {
  ProximityEngine<double> engine;
  const std::string path_a =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  const std::string path_b =
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj");

  // Add the shape (dynamic vs anchored as indicated). Returns a pointer to the
  // underlying collision geometry -- the cached quantity.
  auto add_geometry = [&engine](const Shape& shape, bool is_dynamic = true) {
    const GeometryId id = GeometryId::get_new_id();
    if (is_dynamic) {
      engine.AddDynamicGeometry(shape, {}, id);
    } else {
      engine.AddAnchoredGeometry(shape, {}, id);
    }
    const fcl::CollisionObjectd* obj =
        ProximityEngineTester::GetCollisionObject(engine, id);
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
GTEST_TEST(ProximityEngineTests, ConvexHullCacheScaleIsRespected) {
  ProximityEngine<double> engine;
  const std::string path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  const GeometryId id_C = GeometryId::get_new_id();

  // Each box is placed 10m along a different axis and scaled to double only
  // that axis' half-extent, so the distance from the origin to each box's
  // nearest face is exactly 10 - 2 = 8 m.
  const RigidTransformd X_WA(Vector3d(-10, 0, 0));
  const RigidTransformd X_WB(Vector3d(0, -10, 0));
  const RigidTransformd X_WC(Vector3d(0, 0, -10));

  engine.AddAnchoredGeometry(Convex(path, Vector3d(2.0, 1.0, 1.0)), X_WA, id_A);
  engine.AddAnchoredGeometry(Convex(path, Vector3d(1.0, 2.0, 1.0)), X_WB, id_B);
  engine.AddAnchoredGeometry(Convex(path, Vector3d(1.0, 1.0, 2.0)), X_WC, id_C);

  const std::unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_A, X_WA}, {id_B, X_WB}, {id_C, X_WC}};

  // Threshold chosen between the correct nearest-face distance (8.0 m) and the
  // nearest distance produced by a wrong-scale (x-doubled) hull placed at
  // y/z = -10 m (which would be 9.0 m).
  const double threshold = 8.5;
  const auto results =
      engine.ComputeSignedDistanceToPoint(Vector3d::Zero(), X_WGs, threshold);

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

// TODO(SeanCurtis-TRI): Confirm that SDF data gets computed for Mesh(vtk) and
// all Convex().

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
GTEST_TEST(ProximityEngineTests, ConvexHullCacheMarginIsRespected) {
  ProximityEngine<double> engine;
  const std::string path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  const double kResHint = 0.5;
  const double kE = 1e8;
  const double kMarginValue = 0.1;
  const double kBoxExtent = 2.0;

  // Spatial order A C B; registration order A B C.
  const double kOffset = kBoxExtent + kMarginValue / 2;
  const RigidTransformd X_WA(Vector3d(-kOffset, 0, 0));
  const RigidTransformd X_WC(Vector3d(0, 0, 0));
  const RigidTransformd X_WB(Vector3d(kOffset, 0, 0));

  ProximityProperties props_no_margin;
  props_no_margin.AddProperty(kHydroGroup, kMargin, 0.0);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_no_margin);

  ProximityProperties props_with_margin;
  props_with_margin.AddProperty(kHydroGroup, kMargin, kMarginValue);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_with_margin);

  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  const GeometryId id_C = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Convex(path), X_WA, id_A, props_no_margin);
  engine.AddDynamicGeometry(Convex(path), X_WB, id_B, props_with_margin);
  engine.AddDynamicGeometry(Convex(path), X_WC, id_C, props_no_margin);

  // Call UpdateWorldPoses for the three dynamic geometries. This triggers
  // computeAABB() for B, which reads the shared geometry's aabb_local. In
  // the bug case, C's registration reset that aabb_local and B loses its
  // inflation here.
  engine.UpdateWorldPoses({{id_A, X_WA}, {id_B, X_WB}, {id_C, X_WC}});

  // B's inflated AABB must overlap C; A is far away and never a candidate.
  std::vector<SortedPair<GeometryId>> candidates =
      engine.FindCollisionCandidates();
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
GTEST_TEST(ProximityEngineTests,
           ConvexHullCacheMarginUpdateDoesNotCorruptSiblings) {
  ProximityEngine<double> engine;
  const std::string path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  const double kResHint = 0.5;
  const double kE = 1e8;
  const double kSmallMargin = 0.1;   // A's margin.
  const double kLargeMargin = 0.2;   // B's margin.
  const double kSphereRadius = 0.5;  // Reference geometry radius.

  const double kBoxExtent = 2;
  const double kOffset = kBoxExtent / 2 + kSphereRadius;

  const RigidTransformd X_WA(Vector3d(-kOffset - kSmallMargin / 2, 0, 0));
  const RigidTransformd X_WB(Vector3d(kOffset + kLargeMargin / 2, 0, 0));
  const RigidTransformd X_WS;  // sphere at origin

  ProximityProperties props_small;
  props_small.AddProperty(kHydroGroup, kMargin, kSmallMargin);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_small);

  ProximityProperties props_large;
  props_large.AddProperty(kHydroGroup, kMargin, kLargeMargin);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_large);

  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  const GeometryId id_S = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Convex(path), X_WA, id_A, props_small);
  engine.AddDynamicGeometry(Convex(path), X_WB, id_B, props_large);
  engine.AddAnchoredGeometry(Sphere(kSphereRadius), X_WS, id_S,
                             ProximityProperties());
  engine.UpdateWorldPoses({{id_A, X_WA}, {id_B, X_WB}});

  // Both inflated AABBs must overlap the sphere => 2 candidates.
  ASSERT_EQ(engine.FindCollisionCandidates().size(), 2);

  // Update A to zero margin. B must be unaffected.
  ProximityProperties props_zero;
  props_zero.AddProperty(kHydroGroup, kMargin, 0.0);
  AddCompliantHydroelasticProperties(kResHint, kE, &props_zero);
  const InternalGeometry geo_A(SourceId::get_new_id(),
                               std::make_unique<Convex>(path),
                               FrameId::get_new_id(), id_A, "A", X_WA);
  engine.UpdateRepresentationForNewProperties(geo_A, props_zero);

  // UpdateWorldPoses triggers computeAABB() on all dynamic objects (A and B).
  // In the bug case, B's aabb_local was reset when A's was updated (shared
  // Convexd), so B would lose its inflation here.
  engine.UpdateWorldPoses({{id_A, X_WA}, {id_B, X_WB}});

  // A-sphere overlap is gone; B-sphere overlap must persist.
  EXPECT_EQ(engine.FindCollisionCandidates().size(), 1);
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
GTEST_TEST(ProximityEngineTests, ConvexHullCacheEviction) {
  ProximityEngine<double> engine;
  const std::string path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  const double kResHint = 0.5;
  const double kE = 1e8;

  ProximityProperties props;
  AddCompliantHydroelasticProperties(kResHint, kE, &props);

  const Convex convex_unit(path, 1.0);
  const Convex convex_scaled(path, Vector3d(2, 1, 1));

  const GeometryId id_G1 = GeometryId::get_new_id();
  const GeometryId id_G2 = GeometryId::get_new_id();
  const GeometryId id_G3 = GeometryId::get_new_id();
  engine.AddDynamicGeometry(convex_unit, {}, id_G1, props);
  engine.AddDynamicGeometry(convex_scaled, {}, id_G2, props);
  engine.AddDynamicGeometry(convex_scaled, {}, id_G3, props);

  // Helper that checks the reverse-map invariant: every entry in
  // geometry_to_hull_key has a corresponding valid entry in convex_hull_cache,
  // and the given removed_id is absent.
  auto validate_eviction = [&](GeometryId removed_id, int expected_files,
                               int expected_hulls, const std::string& label) {
    SCOPED_TRACE(label);
    EXPECT_TRUE(
        ProximityEngineTester::geometry_hull_key_absent(engine, removed_id));
    EXPECT_TRUE(
        ProximityEngineTester::geometry_hull_reverse_map_consistent(engine));
    EXPECT_EQ(ProximityEngineTester::convex_hull_cache_file_entries(engine),
              expected_files);
    EXPECT_EQ(ProximityEngineTester::convex_hull_cache_hull_entries(engine),
              expected_hulls);
  };

  // Confirm initial conditions.
  int expected_files = 1;
  int expected_hulls = 2;
  validate_eviction(GeometryId::get_new_id(), expected_files, expected_hulls,
                    "Fake Geometry");

  // Remove G3 — G2 still holds the (2,1,1,0) hull; no eviction expected.
  engine.RemoveGeometry(id_G3, /* is_dynamic= */ true);
  validate_eviction(id_G3, expected_files, expected_hulls, "G3 removed");

  // Remove G2 — cache is now sole owner of (2,1,1,0); sub-entry evicted.
  engine.RemoveGeometry(id_G2, /* is_dynamic= */ true);
  validate_eviction(id_G2, expected_files, --expected_hulls, "G2 removed");

  // Remove G1 — cache is sole owner of (1,1,1,0); fully evicted.
  engine.RemoveGeometry(id_G1, /* is_dynamic= */ true);
  validate_eviction(id_G1, --expected_files, --expected_hulls, "G1 removed");
}

// Tests that the signed distance field (SDF) data computed for an obj correctly
// accounts for its scale factor.
GTEST_TEST(ProximityEngineTest, ProcessMeshSdfDataForObj) {
  ProximityEngine<double> engine;

  const GeometryId g_id = GeometryId::get_new_id();
  const Vector3d scale(2, 3, 4);
  const Mesh mesh(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
                  scale);
  engine.AddAnchoredGeometry(mesh, {}, g_id);

  const TriangleSurfaceMesh<double>* boundary_mesh =
      ProximityEngineTester::get_mesh_distance_boundary(engine, g_id);

  ASSERT_NE(boundary_mesh, nullptr);
  // The cube is 2x2x2. The bounding box of the boundary mesh should be that,
  // scaled by the given scale factors.
  const auto& [_, size] = boundary_mesh->CalcBoundingBox();
  EXPECT_TRUE(CompareMatrices(size, 2 * scale));
}

// Tests simple addition of anchored geometry.
GTEST_TEST(ProximityEngineTests, AddAnchoredGeometry) {
  ProximityEngine<double> engine;
  const Sphere sphere{0.5};
  const RigidTransformd X_WG{Vector3d{1, 2, 3}};
  const GeometryId id = GeometryId::get_new_id();
  engine.AddAnchoredGeometry(sphere, X_WG, id);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 0);
  EXPECT_TRUE(CompareMatrices(
      ProximityEngineTester::GetX_WG(id, false, engine).GetAsMatrix34(),
      X_WG.GetAsMatrix34()));
}

// Tests addition of both dynamic and anchored geometry.
GTEST_TEST(ProximityEngineTests, AddMixedGeometry) {
  ProximityEngine<double> engine;
  const Sphere sphere{0.5};
  const RigidTransformd X_WA{Vector3d{1, 2, 3}};
  const GeometryId id_1 = GeometryId::get_new_id();
  engine.AddAnchoredGeometry(sphere, X_WA, id_1);
  EXPECT_TRUE(CompareMatrices(
      ProximityEngineTester::GetX_WG(id_1, false, engine).GetAsMatrix34(),
      X_WA.GetAsMatrix34()));

  const RigidTransformd X_WD{Vector3d{-1, -2, -3}};
  const GeometryId id_2 = GeometryId::get_new_id();
  engine.AddDynamicGeometry(sphere, X_WD, id_2);
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);
  EXPECT_TRUE(CompareMatrices(
      ProximityEngineTester::GetX_WG(id_2, true, engine).GetAsMatrix34(),
      X_WD.GetAsMatrix34()));
}

// Tests replacing the proximity properties for a given geometry.
GTEST_TEST(ProximityEngineTests, ReplaceProperties) {
  // Some quick aliases to make the tests more compact.
  using PET = ProximityEngineTester;
  const HydroelasticType kUndefined = HydroelasticType::kUndefined;
  const HydroelasticType kRigid = HydroelasticType::kRigid;

  ProximityEngine<double> engine;
  const double radius = 0.5;
  InternalGeometry sphere(SourceId::get_new_id(), make_unique<Sphere>(radius),
                          FrameId::get_new_id(), GeometryId::get_new_id(),
                          "sphere", RigidTransformd());

  // Note: The order of these tests matter; one builds on the next. Re-ordering
  // *may* break the test.

  // Case: throws when the id doesn't refer to a valid geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.UpdateRepresentationForNewProperties(sphere, {}),
      "The proximity engine does not contain a geometry with the id \\d+; its "
      "properties cannot be updated");

  // Case: The new and old properties have no hydroelastic declarations, however
  // it mindlessly attempts to update the hydroelastic representation.
  {
    ProximityProperties props;
    props.AddProperty("foo", "bar", 1.0);
    engine.AddDynamicGeometry(sphere.shape(), {}, sphere.id(), props);
    EXPECT_EQ(PET::hydroelastic_type(sphere.id(), engine), kUndefined);
    DRAKE_EXPECT_NO_THROW(
        engine.UpdateRepresentationForNewProperties(sphere, {}));
    EXPECT_EQ(PET::hydroelastic_type(sphere.id(), engine), kUndefined);
  }

  // Case: The new set has hydroelastic properties, the old does not; change
  // required.
  {
    ProximityProperties props;
    // Pick a characteristic length sufficiently large that we create the
    // coarsest, cheapest mesh possible.
    EXPECT_EQ(PET::hydroelastic_type(sphere.id(), engine), kUndefined);
    props.AddProperty(kHydroGroup, kElastic,
                      std::numeric_limits<double>::infinity());
    AddRigidHydroelasticProperties(3 * radius, &props);
    DRAKE_EXPECT_NO_THROW(
        engine.UpdateRepresentationForNewProperties(sphere, props));
    EXPECT_EQ(PET::hydroelastic_type(sphere.id(), engine), kRigid);
  }

  // Case: The new set does *not* have hydroelastic properties, the old does;
  // this should remove the hydroelastic representation.
  {
    EXPECT_EQ(PET::hydroelastic_type(sphere.id(), engine), kRigid);
    DRAKE_EXPECT_NO_THROW(engine.UpdateRepresentationForNewProperties(
        sphere, ProximityProperties()));
    EXPECT_EQ(PET::hydroelastic_type(sphere.id(), engine), kUndefined);
  }

  // Create a baseline property set that requests a compliant hydroelastic
  // representation, but is not necessarily sufficient to define one.
  ProximityProperties hydro_trigger;
  hydro_trigger.AddProperty(kHydroGroup, kComplianceType,
                            HydroelasticType::kCompliant);

  // Case: New properties request hydroelastic, but they are incomplete and
  // efforts to assign those properties throw.
  {
    ProximityProperties bad_props_no_elasticity(hydro_trigger);
    bad_props_no_elasticity.AddProperty(kHydroGroup, kRezHint, 1.25);
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.UpdateRepresentationForNewProperties(sphere,
                                                    bad_props_no_elasticity),
        "Cannot create soft Sphere; missing the .+ property");

    ProximityProperties bad_props_no_length(hydro_trigger);
    bad_props_no_length.AddProperty(kHydroGroup, kElastic, 5e8);
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.UpdateRepresentationForNewProperties(sphere,
                                                    bad_props_no_length),
        "Cannot create soft Sphere; missing the .+ property");
  }
}

// Removes geometry (dynamic and anchored) from the engine. The test creates
// a _unique_ engine instance with all dynamic or all anchored geometries.
// It is not necessary to create a mixed engine because the two geometry
// types are segregated.
GTEST_TEST(ProximityEngineTests, RemoveGeometry) {
  for (bool is_dynamic : {true, false}) {
    ProximityEngine<double> engine;

    // Removing a geometry that doesn't exist, throws.
    EXPECT_THROW(engine.RemoveGeometry(GeometryId::get_new_id(), true),
                 std::logic_error);

    // Populate the world with three anchored spheres located on the x-axis at
    // x = 0, 2, & 4. With radius of 0.5, they should *not* be colliding.
    Sphere sphere{0.5};
    std::vector<GeometryId> ids;
    std::unordered_map<GeometryId, RigidTransformd> poses;
    for (int i = 0; i < 3; ++i) {
      const GeometryId id = GeometryId::get_new_id();
      ids.push_back(id);
      poses.insert({id, RigidTransformd{Translation3d{i * 2.0, 0, 0}}});
      // Add rigid properties so we can confirm removal of hydroelastic
      // representation. We use rigid here to make sure things get invoked and
      // rely on the implementation of hydroelastic::Geometries to distinguish
      // soft and rigid.
      ProximityProperties props;
      AddRigidHydroelasticProperties(1.0, &props);
      if (is_dynamic) {
        engine.AddDynamicGeometry(sphere, poses[id], id, props);
      } else {
        engine.AddAnchoredGeometry(sphere, poses[id], id, props);
      }
      EXPECT_EQ(ProximityEngineTester::hydroelastic_type(id, engine),
                HydroelasticType::kRigid);
    }
    int expected_count = static_cast<int>(engine.num_geometries());
    EXPECT_EQ(engine.num_geometries(), expected_count);
    EXPECT_EQ(engine.num_anchored(), is_dynamic ? 0 : expected_count);
    EXPECT_EQ(engine.num_dynamic(), is_dynamic ? expected_count : 0);

    engine.UpdateWorldPoses(poses);

    // Remove objects out of order from how they were added. Confirms that the
    // globals are consistent and that the geometry can only be removed once.
    for (int index : {1, 2, 0}) {
      --expected_count;
      const GeometryId remove_id = ids[index];
      engine.RemoveGeometry(remove_id, is_dynamic);
      EXPECT_EQ(engine.num_geometries(), expected_count);
      EXPECT_EQ(engine.num_anchored(), is_dynamic ? 0 : expected_count);
      EXPECT_EQ(engine.num_dynamic(), is_dynamic ? expected_count : 0);
      EXPECT_THROW(engine.RemoveGeometry(remove_id, is_dynamic),
                   std::logic_error);
      EXPECT_EQ(ProximityEngineTester::hydroelastic_type(remove_id, engine),
                HydroelasticType::kUndefined);
    }
  }
}

// Tests for reading .obj files.------------------------------------------------

// Tests exception when we fail to read an .obj file into a Convex.
GTEST_TEST(ProximityEngineTests, FailedParsing) {
  ProximityEngine<double> engine;

  const std::filesystem::path temp_dir = temp_directory();
  // An empty file.
  {
    const std::filesystem::path file = temp_dir / "empty.obj";
    std::ofstream f(file.string());
    f.close();
    Convex convex(file.string());
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.AddDynamicGeometry(convex, {}, GeometryId::get_new_id()),
        ".*cannot be used on a mesh with fewer than three vertices.*");
  }

  // The file does not have OBJ contents.
  {
    const std::filesystem::path file = temp_dir / "not_really_an_obj.obj";
    std::ofstream f(file.string());
    f << "I'm not a valid obj\n";
    f.close();
    Convex convex(file.string());
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.AddDynamicGeometry(convex, {}, GeometryId::get_new_id()),
        ".*cannot be used on a mesh with fewer than three vertices.*");
  }
}

// Tests for copy/move semantics.  ---------------------------------------------

// Tests the copy semantics of the ProximityEngine -- the copy is a complete
// copy. Each CollisionObjectd (and its transform, user data, and AABB) is
// individually duplicated, but the underlying fcl collision geometry is shared
// rather than deep-copied, since FCL geometry is treated as immutable after
// construction. As cloning makes no special effort based on geometry type, we
// can use one or two arbitrary, representative shapes for this test.
GTEST_TEST(ProximityEngineTests, CopySemantics) {
  ProximityEngine<double> ref_engine;
  const RigidTransformd pose = RigidTransformd::Identity();

  // NOTE: The GeometryId values are all lies; the values are arbitrary but
  // do not matter in the context of this test.
  ref_engine.AddAnchoredGeometry(Sphere{0.5}, pose, GeometryId::get_new_id());
  ref_engine.AddDynamicGeometry(Sphere{0.5}, pose, GeometryId::get_new_id());

  ref_engine.AddDynamicGeometry(
      Convex{drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj")},
      pose, GeometryId::get_new_id());

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
  RigidTransformd pose = RigidTransformd::Identity();
  engine.AddAnchoredGeometry(sphere, pose, GeometryId::get_new_id());
  engine.AddDynamicGeometry(sphere, pose, GeometryId::get_new_id());

  ProximityEngine<double> move_construct(std::move(engine));
  EXPECT_EQ(move_construct.num_geometries(), 2);
  EXPECT_EQ(move_construct.num_anchored(), 1);
  EXPECT_EQ(move_construct.num_dynamic(), 1);
  EXPECT_EQ(engine.num_geometries(), 0);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 0);

  ProximityEngine<double> move_assign;
  move_assign = std::move(move_construct);
  EXPECT_EQ(move_assign.num_geometries(), 2);
  EXPECT_EQ(move_assign.num_anchored(), 1);
  EXPECT_EQ(move_assign.num_dynamic(), 1);
  EXPECT_EQ(move_construct.num_geometries(), 0);
  EXPECT_EQ(move_construct.num_anchored(), 0);
  EXPECT_EQ(move_construct.num_dynamic(), 0);
}

// Test harness for testing the ProximityEngine's spatial query functions.
// The tests will examine ProximityEngine's bookkeeping responsibilities for
// each of the spatial query APIs -- the mathematics and correctness of the
// values is deferred to callback implementation tests.
class ProximityEngineQueryTest : public ::testing::Test {
 protected:
  // Adds the given shape as a *dynamic* geometry at the given (optional)
  // position to the test's engine, registering the pose with the global pose
  // lookup table.
  GeometryId AddDynamic(const Shape& shape,
                        const Vector3d& p_WG = Vector3d::Zero(),
                        const ProximityProperties& props = {}) {
    const GeometryId id = GeometryId::get_new_id();
    const RigidTransformd X_WG(p_WG);
    X_WGs_.insert({id, X_WG});
    engine_.AddDynamicGeometry(shape, X_WG, id, props);
    return id;
  }
  // Adds the given shape as an *anchored* geometry at the given (optional)
  // position to the test's engine, registering the pose with the global pose
  // lookup table.
  GeometryId AddAnchored(const Shape& shape,
                         const Vector3d& p_WG = Vector3d::Zero(),
                         const ProximityProperties& props = {}) {
    const GeometryId id = GeometryId::get_new_id();
    const RigidTransformd X_WG(p_WG);
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
        });
    return std::nullopt;
  }

  ProximityEngine<double> engine_;
  unordered_map<GeometryId, RigidTransform<double>> X_WGs_;
};

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
TEST_F(ProximityEngineQueryTest, ComputeSignedDistancePairwiseClosestPoints) {
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
  const GeometryId anchored1 = AddAnchored(sphere, {0, 0, 0});
  // We don't need anchored2's id; it never gets reported.
  AddAnchored(sphere, {0, -d, 0});
  const GeometryId dynamic3 = AddDynamic(sphere, {d, 0, 0});
  const GeometryId dynamic4 = AddDynamic(sphere, {2 * d, 0, 0});

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
TEST_F(ProximityEngineQueryTest, ComputeSignedDistancePairClosestPoints) {
  const Sphere sphere{0.5};
  // Place the spheres apart (not colliding) so the distance query is valid.
  const double d = 2 * sphere.radius() + 0.1;
  const GeometryId id_A = AddDynamic(sphere);
  const GeometryId id_B = AddDynamic(sphere, {d, 0, 0});
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
TEST_F(ProximityEngineQueryTest, ComputeSignedDistanceToPoint) {
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
  const GeometryId dynamic1 = AddDynamic(sphere, {next_x(), 0, 0});
  const GeometryId anchored2 = AddAnchored(sphere, {next_x(), 0, 0});
  const GeometryId dynamic3 = AddDynamic(sphere, {next_x(), 0, 0});
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
  ExcludeCollisionsWithin({dynamic1, anchored2, dynamic1});
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
TEST_F(ProximityEngineQueryTest, ComputeSignedDistanceGeometryToPoint) {
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
  const GeometryId dynamic1 = AddDynamic(sphere, {kD1x, 0.0, 0.0});
  const GeometryId anchored2 = AddAnchored(sphere, {kA2x, 0.0, 0.0});
  const GeometryId dynamic3 = AddDynamic(sphere, {kD3x, 0.0, 0.0});

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
TEST_F(ProximityEngineQueryTest, ComputePointPairPenetration) {
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
  const GeometryId id_A = AddDynamic(sphere, {0, 0, 0});
  const GeometryId id_B = AddDynamic(sphere, {d, 0, 0});

  // Responsibility 2: colliding anchored-anchored pair is NOT reported.
  AddAnchored(sphere, {0, 100, 0});
  AddAnchored(sphere, {d, 100, 0});

  // Responsibility 3: colliding dynamic-anchored pair IS reported.
  const GeometryId id_C = AddDynamic(sphere, {0, 200, 0});
  const GeometryId id_D = AddAnchored(sphere, {d, 200, 0});

  // Responsibility 4: filtered dynamic-dynamic pair is NOT reported.
  const GeometryId id_G = AddDynamic(sphere, {0, 300, 0});
  const GeometryId id_H = AddDynamic(sphere, {d, 300, 0});
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

/* ComputeContactSurfaces() responsibilities:
  1. Empty engine produces no results.
  2. Collision result for dynamic-dynamic pair.
  3. Collision result for anchored-dynamic pair.
  4. No collision result for anchored-anchored pair.
  5. Respect representation format.
  6. Results are always ordered.
  7. Incompatible geometries trigger a throw.
  8. Respect collision filter.
  9. AutoDiff derivatives pass through successfully.
 10. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

         ○○○     ●●●     □□□     ···
       ○     ○ ●     ● □     □ ·     ·     - Prefixes A, D indicated anchored,
     ○   A1  ● ○ D2  □ ●  D3 · □  D5   ·     dynamic.
     ○       ● ○     □ ●     · □       ·   - Suffixes indicate creation order.
       ○ ▲▲▲ ○ ●     ● □     □ ·     ·     - Intersecting pairs: (A1, A4),
       ▲ ○○○ ▲   ●●●     □□□     ···         (A1, D2), (D2, D3), (D3, D5).
     ▲         ▲                           - D5 (labeled "bad" below) has no
     ▲   A4    ▲                             hydro properties and will trigger
       ▲     ▲                               an exception.
         ▲▲▲
  */
TEST_F(ProximityEngineQueryTest, ComputeContactSurfaces) {
  auto eval_dut = [this](HydroelasticContactRepresentation rep) {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputeContactSurfaces(rep, X_WGs_);
  };

  using IdPairs = std::vector<std::pair<GeometryId, GeometryId>>;
  auto extract_ids = [](const vector<ContactSurface<double>>& results) {
    auto ids = results |
               std::views::transform([](const ContactSurface<double>& result) {
                 return std::make_pair(result.id_M(), result.id_N());
               });
    return IdPairs(ids.begin(), ids.end());
  };

  using enum HydroelasticContactRepresentation;

  // (1) - empty engine produces no results.
  ASSERT_TRUE(eval_dut(kTriangle).empty());

  const Sphere sphere(0.5);
  // Distance between sphere centers to guarantee collision.
  const double d = sphere.radius() * 2 * 0.9;

  ProximityProperties props;
  AddCompliantHydroelasticProperties(0.5, 1e-8, &props);
  const GeometryId id_A1 = AddAnchored(sphere, {d, 0, 0}, props);
  const GeometryId id_D2 = AddDynamic(sphere, {2 * d, 0, 0}, props);
  const GeometryId id_D3 = AddDynamic(sphere, {3 * d, 0, 0}, props);
  // We don't need id_A4; it should appear in no results. Its presence would
  // indicate an error.
  AddAnchored(sphere, {d, -d, 0}, props);

  // (2)-(4).
  const vector<ContactSurface<double>> results_tri = eval_dut(kTriangle);
  // We get the (3) A-D and (2) D-D result but not an (4) A-A result.
  ASSERT_EQ(extract_ids(results_tri),
            IdPairs({{id_A1, id_D2}, {id_D2, id_D3}}));

  // (5) - consistent ordering in results.
  const vector<ContactSurface<double>> results_poly = eval_dut(kPolygon);
  ASSERT_EQ(extract_ids(results_poly), extract_ids(results_tri));

  // (6) - representation respected.
  for (const auto& result : results_poly) {
    EXPECT_FALSE(result.is_triangle());
  }
  for (const auto& result : results_tri) {
    EXPECT_TRUE(result.is_triangle());
  }

  // (7) - incompatible geometries trigger a throw; D5 is the "bad" geometry.
  const GeometryId id_bad =
      AddDynamic(sphere, {4 * d, 0, 0});  // No hydro props.
  EXPECT_THROW(eval_dut(kTriangle), std::logic_error);

  // (8) - filters are respected.
  engine_.collision_filter().Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet()),
      [id_D3, id_bad](const GeometrySet&, CollisionFilterScope) {
        return std::unordered_set<GeometryId>{{id_D3, id_bad}};
      });
  ASSERT_EQ(eval_dut(kTriangle).size(), 2);

  // (9) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Seed id_D2's translation with derivatives.
  X_WGs_ad[id_D2] = RigidTransform<AutoDiffXd>{
      math::InitializeAutoDiff(X_WGs_.at(id_D2).translation())};

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  const vector<ContactSurface<AutoDiffXd>> ad_results =
      ad_engine->ComputeContactSurfaces(kTriangle, X_WGs_ad);
  // Still get two results.
  ASSERT_EQ(ad_results.size(), 2);
  // Confirm derivatives on the second result -- not empty and not zero.
  const auto& result = ad_results[1];
  const auto& derivs = result.total_area().derivatives();
  ASSERT_EQ(derivs.size(), 3);
  EXPECT_FALSE(derivs.isZero());
}

/* ComputeContactSurfacesWithFallback() responsibilities:
  1. Empty engine produces no results.
  2. Collision result for dynamic-dynamic pair.
  3. Collision result for anchored-dynamic pair.
  4. No collision result for anchored-anchored pair.
  5. Respect representation format.
  6. Results are always ordered.
  7. No contact surface implies point pair penetration.
  8. Respect collision filter.
  9. AutoDiff derivatives pass through successfully.
 10. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

         ○○○     ●●●     □□□     ···
       ○     ○ ●     ● □     □ ·     ·     - Prefixes A, D indicated anchored,
     ○   A1  ● ○ D2  □ ●  D3 · □  D5   ·     dynamic.
     ○       ● ○     □ ●     · □       ·   - Suffixes indicate creation order.
       ○ ▲▲▲ ○ ●     ● □     □ ·     ·     - Intersecting pairs: (A1, A4),
       ▲ ○○○ ▲   ●●●     □□□     ···         (A1, D2), (D2, D3), (D3, D5).
     ▲         ▲                           - D5 has no hydro properties and will
     ▲   A4    ▲                             lead to a point-pair contact.
       ▲     ▲
         ▲▲▲
  */
TEST_F(ProximityEngineQueryTest, ComputeContactSurfacesWithFallback) {
  vector<ContactSurface<double>> surfaces;
  vector<PenetrationAsPointPair<double>> points;
  auto eval_dut = [this, &points,
                   &surfaces](HydroelasticContactRepresentation rep) {
    surfaces.clear();
    points.clear();
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputeContactSurfacesWithFallback(rep, X_WGs_, &surfaces,
                                                      &points);
  };

  using IdPairs = std::vector<std::pair<GeometryId, GeometryId>>;
  auto surface_ids = [](const vector<ContactSurface<double>>& results) {
    auto ids = results |
               std::views::transform([](const ContactSurface<double>& result) {
                 return std::make_pair(result.id_M(), result.id_N());
               });
    return IdPairs(ids.begin(), ids.end());
  };

  auto point_ids = [](const vector<PenetrationAsPointPair<double>>& results) {
    auto ids = results | std::views::transform(
                             [](const PenetrationAsPointPair<double>& result) {
                               return std::make_pair(result.id_A, result.id_B);
                             });
    return IdPairs(ids.begin(), ids.end());
  };

  using enum HydroelasticContactRepresentation;

  // (1) - empty engine produces no results.
  eval_dut(kTriangle);
  ASSERT_TRUE(surfaces.empty());
  ASSERT_TRUE(points.empty());

  const Sphere sphere(0.5);
  // Distance between sphere centers to guarantee collision.
  const double d = sphere.radius() * 2 * 0.9;

  ProximityProperties props;
  AddCompliantHydroelasticProperties(0.5, 1e-8, &props);
  const GeometryId id_A1 = AddAnchored(sphere, {d, 0, 0}, props);
  const GeometryId id_D2 = AddDynamic(sphere, {2 * d, 0, 0}, props);
  const GeometryId id_D3 = AddDynamic(sphere, {3 * d, 0, 0}, props);
  // We don't save id_A4; it's never referenced.
  AddAnchored(sphere, {d, -d, 0}, props);

  // (2)-(4).
  // We get the (3) A-D and (2) D-D result but not an (4) A-A result.
  eval_dut(kTriangle);
  const vector<ContactSurface<double>> results_tri = surfaces;
  ASSERT_EQ(surface_ids(results_tri),
            IdPairs({{id_A1, id_D2}, {id_D2, id_D3}}));
  ASSERT_TRUE(points.empty());

  // (5) - consistent ordering in results.
  eval_dut(kPolygon);
  const vector<ContactSurface<double>> results_poly = surfaces;
  ASSERT_EQ(surface_ids(results_poly), surface_ids(results_tri));
  ASSERT_TRUE(points.empty());

  // (6) - representation respected.
  for (const auto& result : results_poly) {
    EXPECT_FALSE(result.is_triangle());
  }
  for (const auto& result : results_tri) {
    EXPECT_TRUE(result.is_triangle());
  }

  // (7) - hydro-incompatible geometries produces point pair.
  const GeometryId id_D5 =
      AddDynamic(sphere, {4 * d, 0, 0});  // No hydro props.
  eval_dut(kTriangle);
  ASSERT_EQ(surface_ids(surfaces), IdPairs({{id_A1, id_D2}, {id_D2, id_D3}}));
  ASSERT_EQ(point_ids(points), IdPairs({{id_D3, id_D5}}));

  // (8) - filters are respected.
  const FilterId filter_id = engine_.collision_filter().ApplyTransient(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet()),
      [id_D2, id_D3, id_D5](const GeometrySet&, CollisionFilterScope) {
        return std::unordered_set<GeometryId>{{id_D2, id_D3, id_D5}};
      });
  eval_dut(kTriangle);
  // The filter removed one contact surface and one point pair.
  ASSERT_EQ(surfaces.size(), 1);
  ASSERT_EQ(points.size(), 0);
  engine_.collision_filter().RemoveDeclaration(filter_id);

  // (9) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Seed id_D3's translation with derivatives - the second contact surface and
  // only point pair both involve D3.
  X_WGs_ad[id_D3] = RigidTransform<AutoDiffXd>{
      math::InitializeAutoDiff(X_WGs_.at(id_D3).translation())};

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  vector<ContactSurface<AutoDiffXd>> surfaces_ad;
  vector<PenetrationAsPointPair<AutoDiffXd>> points_ad;
  ad_engine->ComputeContactSurfacesWithFallback(kTriangle, X_WGs_ad,
                                                &surfaces_ad, &points_ad);
  // Still get two results.
  ASSERT_EQ(surfaces_ad.size(), 2);
  ASSERT_EQ(points_ad.size(), 1);
  // Confirm derivatives on the surface and point result.
  const auto& surface = surfaces_ad[1];
  const auto& surf_deriv = surface.total_area().derivatives();
  ASSERT_EQ(surf_deriv.size(), 3);
  EXPECT_FALSE(surf_deriv.isZero());

  const auto& point = points_ad[0];
  const auto& point_deriv = point.depth.derivatives();
  ASSERT_EQ(point_deriv.size(), 3);
  EXPECT_FALSE(point_deriv.isZero());
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
TEST_F(ProximityEngineQueryTest, FindCollisionCandidates) {
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
  const GeometryId anchored2 = AddAnchored(sphere, {0, -d, 0});
  ASSERT_TRUE(eval_dut().empty());

  // (3) - dynamic-dynamic pairs whose AABBs overlap ARE candidates.
  const GeometryId dynamic3 = AddDynamic(sphere, {2, 0, 0});
  const GeometryId dynamic4 = AddDynamic(sphere, {2, -d, 0});
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
TEST_F(ProximityEngineQueryTest, HasCollisions) {
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
  const GeometryId dynamic3 = AddDynamic(sphere, Vector3d(0, 1.5 * d, 0));
  const GeometryId dynamic4 = AddDynamic(sphere, Vector3d(d, 1.5 * d, 0));

  ASSERT_TRUE(eval_dut());

  // (4) - Filtering the dynamic pair, removes the only collision.
  ExcludeCollisionsWithin({dynamic3, dynamic4});
  ASSERT_FALSE(eval_dut());

  // (5) - dynamic-anchored are reported.
  // Place D5 in pose D5₁.
  const GeometryId dynamic5 = AddDynamic(sphere, Vector3d(0, -d, 0));
  ASSERT_TRUE(eval_dut());

  // (6) - Weird special case. There was a legacy bug (reported in issue #23406)
  // where collisions between dynamic geometries were erased if there were
  // no anchored-dynamic collisions, but there were overlapping bounding volumes
  // between anchored and dynamic geometries. So, we move D5 to position D5₂
  X_WGs_[dynamic5] = RigidTransformd(Vector3d(d / 2, d, 0));

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

// TODO(SeanCurtis-TRI): All of the FCL-based queries should have *limited*
//  testing in proximity engine. They should only test the following:
//  Successful evaluation between two dynamic shapes and between a dynamic
//  and anchored shape. Those simple tests confirm ProximityEngine handles its
//  responsibility correctly; it calls the appropriate broadphase methods on
//  FCL. Unit tests on ProximityEngine for the FCL-based query methods should
//  *not* be concerned with the correctness of the results' values -- that lies
//  in the responsibility of the callback and underlying geometric algorithms.
//  They were originally created here because FCL wasn't completely trusted. So,
//  create a set of tests that can be evaluated on each of the FCL-based queries
//  that performs those two tests.

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
GTEST_TEST(ProximityEngineTests, AnchoredBroadPhaseInitialization) {
  ProximityEngine<double> engine;
  GeometryId id_D = GeometryId::get_new_id();
  GeometryId id_A = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Sphere(0.5), {}, id_D);

  RigidTransformd X_WA{Translation3d{-3, 0, 0}};
  engine.AddAnchoredGeometry(Sphere(0.5), X_WA, id_A);

  RigidTransformd X_WD{Translation3d{-3, 0.75, 0}};

  std::unordered_map<GeometryId, math::RigidTransform<double>> X_WGs{
      {id_A, X_WA}, {id_D, X_WD}};
  engine.UpdateWorldPoses(X_WGs);

  EXPECT_TRUE(engine.HasCollisions());

  // Confirm that it survives copying.
  ProximityEngine<double> engine_copy(engine);
  engine_copy.UpdateWorldPoses({{id_D, X_WD}});
  EXPECT_TRUE(engine_copy.HasCollisions());
}

// Test fixture for deformable contact.
class ProximityEngineDeformableContactTest : public testing::Test {
 protected:
  static constexpr double kSphereRadius = 1;

  ProximityEngineDeformableContactTest() {
    const internal::deformable::Geometries& geometries =
        deformable_contact_geometries();
    internal::deformable::GeometriesTester::disable_rigid_geometry_deferral(
        const_cast<internal::deformable::Geometries*>(&geometries));
  }

  // Makes proximity properties that allows a geometry to be registered as a
  // rigid geometry for deformable contact.
  ProximityProperties MakeProximityPropsForRigidGeometry(
      double resolution_hint) {
    ProximityProperties props;
    props.AddProperty(internal::kHydroGroup, internal::kRezHint,
                      resolution_hint);
    props.AddProperty(internal::kHydroGroup, internal::kElastic, 1e6);
    return props;
  }

  // Returns the data structure for holding geometries participating in
  // deformable contact.
  const deformable::Geometries& deformable_contact_geometries() const {
    return ProximityEngineTester::get_deformable_contact_geometries(engine_);
  }

  // Returns the deformable geometry with the given id.
  // @pre There exists a deformable geometry with the given id.
  const internal::deformable::DeformableGeometry& deformable_geometry(
      GeometryId id) {
    return internal::deformable::GeometriesTester::get_deformable_geometry(
        deformable_contact_geometries(), id);
  }

  // Returns the rigid (non-deformable) geometry with the given id.
  // @pre There exists a rigid geometry with the given id.
  const internal::deformable::RigidGeometry& rigid_geometry(GeometryId id) {
    return internal::deformable::GeometriesTester::get_rigid_geometry(
        deformable_contact_geometries(), id);
  }

  // Given the volume mesh of a deformable geometry, registers it with the
  // proximity engine and stores the driven mesh data in the testing class.
  void AddDeformableGeometry(const VolumeMesh<double>& mesh, GeometryId id) {
    std::vector<int> surface_vertices;
    std::vector<int> surface_tri_to_volume_tet;
    TriangleSurfaceMesh<double> surface_mesh =
        ConvertVolumeToSurfaceMeshWithBoundaryVertices(
            mesh, &surface_vertices, &surface_tri_to_volume_tet);
    engine_.AddDeformableGeometry(mesh, surface_mesh, surface_vertices,
                                  surface_tri_to_volume_tet, id);

    VertexSampler vertex_sampler(std::move(surface_vertices), mesh);
    std::vector<DrivenTriangleMesh> driven_meshes;
    driven_meshes.emplace_back(vertex_sampler, surface_mesh);
    driven_mesh_data_.SetMeshes(id, std::move(driven_meshes));
  }

  // Registers a deformable sphere with the proximity engine and stores the
  // driven mesh data in the testing class. Returns the id of the
  // newly registered geometry.
  GeometryId AddDeformableSphere() {
    constexpr double kResolutionHint = 0.5;
    Sphere sphere(kSphereRadius);
    const VolumeMesh<double> input_mesh = MakeSphereVolumeMesh<double>(
        sphere, kResolutionHint, TessellationStrategy::kDenseInteriorVertices);
    const GeometryId id = GeometryId::get_new_id();
    AddDeformableGeometry(input_mesh, id);
    return id;
  }

  // Tests registering and removing a rigid (non-deformable) geometry in the
  // ProximityEngine by performing the following checks.
  //  1. Add the given `shape` as a dynamic geometry.
  //  2. Add the given `shape` as an anchored geometry.
  //  3. If the geometry is successfully registered, verify that the pose of the
  //     geometry is correctly recorded.
  //  4. If the geometry is successfully registered, verify it can also be
  //     successfully be removed via RemoveGeometry.
  // @param shape           The shape to be registered.
  // @param props           The proximity properties of the shape being
  //                        registered. The properties may or may not actually
  //                        require a rigid geometry for deformable contact.
  // @param expected_status Whether the registration is expected to be
  //                        successful.
  template <typename Shape>
  void TestRigidRegistrationAndRemoval(const Shape& shape,
                                       const ProximityProperties& props,
                                       bool expected_status) {
    // Arbitrary pose of the rigid geometry.
    RollPitchYawd rpy_WG(1, 2, 3);
    Vector3d p_WG(4, 5, 6);
    RigidTransformd X_WG(rpy_WG, p_WG);
    // Register as dynamic geometry.
    {
      const GeometryId id = GeometryId::get_new_id();
      engine_.AddDynamicGeometry(shape, X_WG, id, props);
      const bool registered = deformable_contact_geometries().is_rigid(id);
      EXPECT_EQ(registered, expected_status);
      if (registered) {
        const deformable::RigidGeometry& geometry = rigid_geometry(id);
        const RigidTransformd X_WG_registered = geometry.pose_in_world();
        EXPECT_TRUE(X_WG.IsExactlyEqualTo(X_WG_registered));
        EXPECT_TRUE(deformable_contact_geometries().is_rigid(id));
        engine_.RemoveGeometry(id, true);
        EXPECT_FALSE(deformable_contact_geometries().is_rigid(id));
      }
    }
    // Register as anchored geometry.
    {
      const GeometryId id = GeometryId::get_new_id();
      engine_.AddAnchoredGeometry(shape, X_WG, id, props);
      const bool registered = deformable_contact_geometries().is_rigid(id);
      EXPECT_EQ(registered, expected_status);
      if (registered) {
        const deformable::RigidGeometry& geometry = rigid_geometry(id);
        const RigidTransformd X_WG_registered = geometry.pose_in_world();
        EXPECT_TRUE(X_WG.IsExactlyEqualTo(X_WG_registered));
        EXPECT_TRUE(deformable_contact_geometries().is_rigid(id));
        engine_.RemoveGeometry(id, false);
        EXPECT_FALSE(deformable_contact_geometries().is_rigid(id));
      }
    }
  }

  ProximityEngine<double> engine_;
  internal::DrivenMeshData driven_mesh_data_;
};

// Tests that registration of supported rigid (non-deformable) geometries make
// their way to deformable::Geometries if the resolution hint property is valid.
// Also verifies that no geometry is added if the resolution hint property is
// missing.
TEST_F(ProximityEngineDeformableContactTest, AddSupportedRigidGeometries) {
  ProximityProperties valid_props = MakeProximityPropsForRigidGeometry(1.0);
  ProximityProperties empty_props;
  {
    Sphere sphere(0.5);
    TestRigidRegistrationAndRemoval(sphere, valid_props, true);
    TestRigidRegistrationAndRemoval(sphere, empty_props, false);
  }
  {
    Box box = Box::MakeCube(1.0);
    TestRigidRegistrationAndRemoval(box, valid_props, true);
    TestRigidRegistrationAndRemoval(box, empty_props, false);
  }
  {
    Cylinder cylinder(1.0, 2.0);
    TestRigidRegistrationAndRemoval(cylinder, valid_props, true);
    TestRigidRegistrationAndRemoval(cylinder, empty_props, false);
  }
  {
    Capsule capsule(1.0, 2.0);
    TestRigidRegistrationAndRemoval(capsule, valid_props, true);
    TestRigidRegistrationAndRemoval(capsule, empty_props, false);
  }
  {
    Ellipsoid ellipsoid(1.0, 2.0, 3.0);
    TestRigidRegistrationAndRemoval(ellipsoid, valid_props, true);
    TestRigidRegistrationAndRemoval(ellipsoid, empty_props, false);
  }
  {
    Convex convex(
        drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj"));
    TestRigidRegistrationAndRemoval(convex, valid_props, true);
    TestRigidRegistrationAndRemoval(convex, empty_props, false);
  }
  {
    Mesh mesh(drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj"));
    TestRigidRegistrationAndRemoval(mesh, valid_props, true);
    TestRigidRegistrationAndRemoval(mesh, empty_props, false);
  }
}

// Tests that registration of geometries supported by ProximityEngine but not
// supported by deformable contact (aka HalfSpace) doesn't throw.
TEST_F(ProximityEngineDeformableContactTest, AddUnsupportedRigidGeometries) {
  ProximityProperties valid_props = MakeProximityPropsForRigidGeometry(1.0);
  HalfSpace half_space;
  EXPECT_NO_THROW(
      TestRigidRegistrationAndRemoval(half_space, valid_props, false));
}

// Tests that replacing properties for rigid (non-deformable geometries) yields
// expected behaviors.
TEST_F(ProximityEngineDeformableContactTest, ReplacePropertiesRigid) {
  const RigidTransform<double> X_FG(RollPitchYawd(1, 2, 3), Vector3d(4, 5, 6));
  InternalGeometry sphere(
      SourceId::get_new_id(), make_unique<Sphere>(kSphereRadius),
      FrameId::get_new_id(), GeometryId::get_new_id(), "sphere", X_FG);

  // Note: The order of these tests matter; one builds on the next. Re-ordering
  // *may* break the test.

  // Case: throws when the id doesn't refer to a valid geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.UpdateRepresentationForNewProperties(sphere, {}),
      "The proximity engine does not contain a geometry with the id \\d+; its "
      "properties cannot be updated");

  // Case: The new and old properties have no relevant deformable contact
  // declarations (i.e. no resolution hint).
  {
    ProximityProperties props;
    props.AddProperty("foo", "bar", 1.0);
    engine_.AddDynamicGeometry(sphere.shape(), {}, sphere.id(), props);
    EXPECT_FALSE(deformable_contact_geometries().is_rigid(sphere.id()));
    DRAKE_EXPECT_NO_THROW(
        engine_.UpdateRepresentationForNewProperties(sphere, {}));
    EXPECT_FALSE(deformable_contact_geometries().is_rigid(sphere.id()));
  }

  // Case: The new properties have resolution hint while the old do not; this
  // should add a new deformable contact rigid representation.
  {
    ProximityProperties props = MakeProximityPropsForRigidGeometry(1.0);
    DRAKE_EXPECT_NO_THROW(
        engine_.UpdateRepresentationForNewProperties(sphere, props));
    EXPECT_TRUE(deformable_contact_geometries().is_rigid(sphere.id()));
  }

  // Case: Both the old and the new properties have resolution hint, but with
  // different values; the rigid representation would have a new mesh but pose
  // of the geometry won't change.
  {
    const deformable::RigidGeometry geometry_old =
        deformable::GeometriesTester::get_rigid_geometry(
            deformable_contact_geometries(), sphere.id());
    const RigidTransformd X_WG_old = geometry_old.pose_in_world();
    const int num_vertices_old = geometry_old.mesh().mesh().num_vertices();

    ProximityProperties props = MakeProximityPropsForRigidGeometry(0.5);
    engine_.UpdateRepresentationForNewProperties(sphere, props);

    const deformable::RigidGeometry geometry_new =
        deformable::GeometriesTester::get_rigid_geometry(
            deformable_contact_geometries(), sphere.id());
    const RigidTransformd X_WG_new = geometry_new.pose_in_world();
    const int num_vertices_new = geometry_new.mesh().mesh().num_vertices();

    EXPECT_TRUE(deformable_contact_geometries().is_rigid(sphere.id()));
    EXPECT_TRUE(X_WG_new.IsExactlyEqualTo(X_WG_old));
    // The inequality in number of vertices of the rigid mesh indicates the mesh
    // has changed.
    EXPECT_NE(num_vertices_old, num_vertices_new);
  }

  // Case: The new properties don't have resolution hint while the old do;
  // this should remove the deformable contact representation.
  {
    EXPECT_TRUE(deformable_contact_geometries().is_rigid(sphere.id()));
    DRAKE_EXPECT_NO_THROW(engine_.UpdateRepresentationForNewProperties(
        sphere, ProximityProperties()));
    EXPECT_FALSE(deformable_contact_geometries().is_rigid(sphere.id()));
  }
}

// Tests that replacing properties for deformable geometries yields expected
// behaviors.
TEST_F(ProximityEngineDeformableContactTest, ReplacePropertiesDeformable) {
  InternalGeometry sphere(SourceId::get_new_id(),
                          make_unique<Sphere>(kSphereRadius),
                          FrameId::get_new_id(), GeometryId::get_new_id(),
                          "sphere", RigidTransformd{}, 1.0);

  // Case: throws when the id doesn't refer to a valid geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.UpdateRepresentationForNewProperties(sphere, {}),
      "The proximity engine does not contain a geometry with the id \\d+; its "
      "properties cannot be updated");

  // Case: The new and old properties are both valid proximity properties. The
  // update is a no-op in this case.
  ProximityProperties props;
  AddDeformableGeometry(*sphere.reference_mesh(), sphere.id());
  EXPECT_TRUE(deformable_contact_geometries().is_deformable(sphere.id()));
  const internal::deformable::DeformableGeometry& old_geometry =
      deformable_geometry(sphere.id());
  DRAKE_EXPECT_NO_THROW(
      engine_.UpdateRepresentationForNewProperties(sphere, {}));
  EXPECT_TRUE(deformable_contact_geometries().is_deformable(sphere.id()));
  const internal::deformable::DeformableGeometry& new_geometry =
      deformable_geometry(sphere.id());
  // Verify that the content didn't change.
  EXPECT_TRUE(old_geometry.deformable_volume().mesh().Equal(
      new_geometry.deformable_volume().mesh()));
  EXPECT_TRUE(old_geometry.deformable_volume().bvh().Equal(
      new_geometry.deformable_volume().bvh()));
  EXPECT_TRUE(old_geometry.CalcSignedDistanceField().Equal(
      new_geometry.CalcSignedDistanceField()));
  // Verify that the address didn't change either; so it's indeed an no-op.
  EXPECT_EQ(&old_geometry.deformable_volume().mesh(),
            &new_geometry.deformable_volume().mesh());
  EXPECT_EQ(&old_geometry.deformable_volume().bvh(),
            &new_geometry.deformable_volume().bvh());
}

TEST_F(ProximityEngineDeformableContactTest, AddAndRemoveDeformableGeometry) {
  const GeometryId id = AddDeformableSphere();
  EXPECT_TRUE(deformable_contact_geometries().is_deformable(id));
  engine_.RemoveDeformableGeometry(id);
  EXPECT_FALSE(deformable_contact_geometries().is_deformable(id));
  DRAKE_EXPECT_THROWS_MESSAGE(engine_.RemoveDeformableGeometry(id),
                              ".*does not contain a deformable geometry.*");
}

TEST_F(ProximityEngineDeformableContactTest, UpdatePose) {
  ProximityProperties props = MakeProximityPropsForRigidGeometry(1.0);
  const GeometryId id = GeometryId::get_new_id();
  // Arbitrary initial pose of the rigid geometry.
  RollPitchYawd rpy(1, 2, 3);
  Vector3d translation(4, 5, 6);
  RigidTransformd pose(rpy, translation);
  engine_.AddDynamicGeometry(Sphere(1.0), pose, id, props);

  RigidTransformd X_WG(RollPitchYawd(0.1, 0.2, 0.3), Vector3d(0.4, 0.5, 0.6));
  engine_.UpdateWorldPoses({{id, X_WG}});
  EXPECT_TRUE(rigid_geometry(id).pose_in_world().IsExactlyEqualTo(X_WG));
}

TEST_F(ProximityEngineDeformableContactTest, UpdateDeformablePositions) {
  const GeometryId id = AddDeformableSphere();
  const VolumeMesh<double>& mesh =
      deformable_geometry(id).deformable_volume().mesh();
  const int num_vertices = mesh.num_vertices();
  /* Update the vertex positions to some arbitrary value. */
  const VectorX<double> q =
      VectorX<double>::LinSpaced(3 * num_vertices, 0.0, 1.0);
  engine_.UpdateDeformableVertexPositions({{id, q}},
                                          driven_mesh_data_.driven_meshes());
  const VolumeMesh<double>& deformed_mesh =
      deformable_geometry(id).deformable_volume().mesh();
  for (int i = 0; i < num_vertices; ++i) {
    const Vector3d& q_MV = deformed_mesh.vertex(i);
    const Vector3d& expected_q_MV = q.segment<3>(3 * i);
    EXPECT_EQ(q_MV, expected_q_MV);
  }
}

// Verify that ProximityEngine is properly invoking the lower-level collision
// query code by verifying a few necessary conditions.
TEST_F(ProximityEngineDeformableContactTest, ComputeDeformableContact) {
  const GeometryId deformable_id = AddDeformableSphere();

  // Add a rigid sphere partially overlapping the deformable sphere.
  const GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties props = MakeProximityPropsForRigidGeometry(1.0);
  const Vector3d p_WR = Vector3d(kSphereRadius, 0, 0);
  engine_.AddDynamicGeometry(Sphere(kSphereRadius), RigidTransformd(p_WR),
                             rigid_id, props);

  // Verify the two spheres are in contact.
  DeformableContact<double> contact_data;
  engine_.ComputeDeformableContact(&contact_data);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 1);
  DeformableContactSurface<double> contact_surface =
      contact_data.contact_surfaces()[0];
  EXPECT_EQ(contact_surface.id_A(), deformable_id);
  EXPECT_EQ(contact_surface.id_B(), rigid_id);

  // Move the deformable geometry away so that the two geometries are no longer
  // in contact.
  const Vector3d offset_W(10, 0, 0);
  const VolumeMesh<double>& mesh =
      deformable_geometry(deformable_id).deformable_volume().mesh();
  Eigen::VectorXd q_WD(3 * mesh.num_vertices());
  for (int i = 0; i < mesh.num_vertices(); ++i) {
    q_WD.segment<3>(3 * i) = mesh.vertex(i) + offset_W;
  }
  std::unordered_map<GeometryId, Eigen::VectorXd> geometry_id_to_q_WD;
  geometry_id_to_q_WD.insert({deformable_id, q_WD});
  driven_mesh_data_.SetControlMeshPositions(geometry_id_to_q_WD);
  engine_.UpdateDeformableVertexPositions(geometry_id_to_q_WD,
                                          driven_mesh_data_.driven_meshes());
  engine_.ComputeDeformableContact(&contact_data);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 0);

  // Shift the rigid geometry by the same offset and they are again in
  // contact.
  std::unordered_map<GeometryId, RigidTransformd> geometry_id_to_world_poses;
  geometry_id_to_world_poses.insert(
      {rigid_id, RigidTransformd(p_WR + offset_W)});
  engine_.UpdateWorldPoses(geometry_id_to_world_poses);
  engine_.ComputeDeformableContact(&contact_data);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 1);
  contact_surface = contact_data.contact_surfaces()[0];
  EXPECT_EQ(contact_surface.id_A(), deformable_id);
  EXPECT_EQ(contact_surface.id_B(), rigid_id);
}

GTEST_TEST(ProximityEngineTests, NeedsConvexHull) {
  ProximityEngine<double> engine;

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
    InternalGeometry geo(s_id, std::move(shape), f_id, g_id, "n", {});
    EXPECT_FALSE(engine.NeedsConvexHull(geo));
  }

  // Rigid shapes that *do* require convex hulls.
  EXPECT_TRUE(engine.NeedsConvexHull(InternalGeometry(
      s_id, make_unique<Mesh>("unimportant"), f_id, g_id, "n", {})));
  EXPECT_TRUE(engine.NeedsConvexHull(InternalGeometry(
      s_id, make_unique<Convex>("unimportant"), f_id, g_id, "n", {})));

  // Being deformable would eliminate the need for a convex hull.
  EXPECT_FALSE(engine.NeedsConvexHull(InternalGeometry(
      s_id,
      make_unique<Mesh>(
          FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk")),
      f_id, g_id, "n", {}, 1.0)));
}

// ProximityEngine creates fcl::Convexd for all Mesh and Convex. For Convex,
// it's part of the contract. For Mesh, it is the current handicapped
// implementation.
//
// This confirms that the input mesh is ignored in favor of the convex hull
// provided to ProximityEngine.
GTEST_TEST(ProximityEngineTests, ImplementedAsFclConvex) {
  ProximityEngine<double> engine;

  // This mesh has 8 small wedges jammed into the corners of a cube 2-units on
  // the side, centered on the origin. It is decidedly non-convex.
  // In collision queries, it should respond like a solid cube.
  const std::string obj_path =
      FindResourceOrThrow("drake/geometry/test/cube_corners.obj");

  auto expect_fcl_convex_is_cube = [&engine](GeometryId id) {
    const fcl::CollisionObjectd* object =
        ProximityEngineTester::GetCollisionObject(engine, id);
    DRAKE_DEMAND(object != nullptr);
    ASSERT_EQ(object->getNodeType(), fcl::GEOM_CONVEX);
    const fcl::Convexd* fcl_shape =
        dynamic_cast<const fcl::Convexd*>(object->collisionGeometry().get());
    DRAKE_DEMAND(fcl_shape != nullptr);
    // We get the tet's 4 vertices and 4 faces.
    EXPECT_EQ(fcl_shape->getVertices().size(), 8);
    EXPECT_EQ(fcl_shape->getFaceCount(), 6);
  };

  {
    SCOPED_TRACE("Mesh as fcl::Convexd");
    const GeometryId id = GeometryId::get_new_id();
    engine.AddAnchoredGeometry(Mesh(obj_path), {}, id, {});

    expect_fcl_convex_is_cube(id);
  }
  {
    SCOPED_TRACE("Convex as fcl::Convexd");
    const GeometryId id = GeometryId::get_new_id();
    engine.AddAnchoredGeometry(Convex(obj_path), {}, id, {});

    expect_fcl_convex_is_cube(id);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
