#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
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

constexpr double kInf = std::numeric_limits<double>::infinity();

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

// Adds a single shape to the given engine with the indicated anchored/dynamic
// configuration and compliant type.
std::pair<GeometryId, RigidTransformd> AddShape(ProximityEngine<double>* engine,
                                                const Shape& shape,
                                                bool is_anchored,
                                                HydroelasticType hydro_type,
                                                const Vector3d& p_S1S2_W) {
  RigidTransformd X_WS = RigidTransformd(p_S1S2_W);
  const GeometryId id_S = GeometryId::get_new_id();
  // We'll mindlessly provide edge_length; even if the shape doesn't require
  // it.
  const double edge_length = 0.5;
  ProximityProperties properties;
  using enum HydroelasticType;
  switch (hydro_type) {
    case kUndefined:
      // Do nothing.
      break;
    case kRigid:
      AddRigidHydroelasticProperties(edge_length, &properties);
      break;
    case kCompliant:
      AddCompliantHydroelasticProperties(edge_length, 1e8, &properties);
      properties.AddProperty(kHydroGroup, kSlabThickness, 1.0);
      break;
  }
  if (is_anchored) {
    engine->AddAnchoredGeometry(shape, X_WS, id_S, properties);
  } else {
    engine->AddDynamicGeometry(shape, X_WS, id_S, properties);
  }
  return std::make_pair(id_S, X_WS);
}

unordered_map<GeometryId, RigidTransformd> PopulateEngine(
    ProximityEngine<double>* engine, const Shape& shape1, bool anchored1,
    HydroelasticType type1, const Shape& shape2, bool anchored2,
    HydroelasticType type2, const Vector3d& p_S1S2_W = Vector3d(0, 0, 0)) {
  unordered_map<GeometryId, RigidTransformd> X_WGs;
  RigidTransformd X_WG;
  GeometryId id1, id2;
  std::tie(id1, X_WG) =
      AddShape(engine, shape1, anchored1, type1, Vector3d::Zero());
  X_WGs.insert({id1, X_WG});
  std::tie(id2, X_WG) = AddShape(engine, shape2, anchored2, type2, p_S1S2_W);
  X_WGs.insert({id2, X_WG});
  engine->UpdateWorldPoses(X_WGs);
  return X_WGs;
}

// The autodiff support is independent of what the contact surface mesh
// representation is; so we'll simply use kTriangle.
GTEST_TEST(ProximityEngineTest, ComputeContactSurfacesAutodiffSupport) {
  using enum HydroelasticType;
  const bool anchored{true};
  const Sphere sphere{0.2};
  const Mesh mesh(
      drake::FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"));

  // Case: Compliant sphere and rigid mesh with AutoDiffXd -- confirm the
  // contact surface has derivatives.
  {
    ProximityEngine<double> engine_d;
    const auto X_WGs_d = PopulateEngine(&engine_d, sphere, anchored, kCompliant,
                                        mesh, !anchored, kRigid);

    const auto engine_ad = engine_d.ToScalarType<AutoDiffXd>();
    unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
    bool added_derivatives = false;
    for (const auto& [id, X_WG_d] : X_WGs_d) {
      if (!added_derivatives) {
        // We'll set the derivatives on p_WGo for the first geometry; all others
        // we'll pass through.
        const Vector3<AutoDiffXd> p_WGo =
            math::InitializeAutoDiff(X_WG_d.translation());
        X_WGs_ad[id] = RigidTransform<AutoDiffXd>(
            X_WG_d.rotation().cast<AutoDiffXd>(), p_WGo);
        added_derivatives = true;
      } else {
        X_WGs_ad[id] = RigidTransform<AutoDiffXd>(X_WG_d.GetAsMatrix34());
      }
    }

    std::vector<ContactSurface<AutoDiffXd>> surfaces;
    std::vector<PenetrationAsPointPair<AutoDiffXd>> point_pairs;
    // We assume that ComputeContactSurfacesWithFallback() exercises the same
    // code as ComputeContactSurfaces(); they both pass through the hydroelastic
    // calculator. So, exercising one is "sufficient". If they ever deviate in
    // execution (i.e., there were to no longer share the same calculator), this
    // test would have to be elaborated.
    engine_ad->ComputeContactSurfacesWithFallback(
        HydroelasticContactRepresentation::kTriangle, X_WGs_ad, &surfaces,
        &point_pairs);
    EXPECT_EQ(surfaces.size(), 1);
    EXPECT_EQ(point_pairs.size(), 0);
    // We'll poke *one* quantity of the surface mesh to confirm it has
    // derivatives. We won't consider the *value*, just the existence as proof
    // that it has been wired up to code that has already tested value.
    EXPECT_EQ(surfaces[0].tri_mesh_W().vertex(0).x().derivatives().size(), 3);
  }

  // Case: Rigid sphere and hydro-undefined mesh with AutoDiffXd --
  // rigid-undefined contact would result in a point pair.
  {
    ProximityEngine<double> engine_d;
    using enum HydroelasticType;
    const auto X_WGs_d = PopulateEngine(&engine_d, sphere, false, kRigid,
                                        sphere, false, kUndefined);
    const auto engine_ad = engine_d.ToScalarType<AutoDiffXd>();
    unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
    for (const auto& [id, X_WG_d] : X_WGs_d) {
      X_WGs_ad[id] = RigidTransform<AutoDiffXd>(X_WG_d.GetAsMatrix34());
    }

    std::vector<ContactSurface<AutoDiffXd>> surfaces;
    std::vector<PenetrationAsPointPair<AutoDiffXd>> point_pairs;
    engine_ad->ComputeContactSurfacesWithFallback(
        HydroelasticContactRepresentation::kTriangle, X_WGs_ad, &surfaces,
        &point_pairs);
    EXPECT_EQ(surfaces.size(), 0);
    EXPECT_EQ(point_pairs.size(), 1);
  }

  // Case: Rigid sphere and mesh with AutoDiffXd -- rigid-rigid contact would
  // result in a point pair.
  {
    ProximityEngine<double> engine_d;
    const auto X_WGs_d =
        PopulateEngine(&engine_d, sphere, false, kRigid, sphere, false, kRigid);
    const auto engine_ad = engine_d.ToScalarType<AutoDiffXd>();
    unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
    for (const auto& [id, X_WG_d] : X_WGs_d) {
      X_WGs_ad[id] = RigidTransform<AutoDiffXd>(X_WG_d.GetAsMatrix34());
    }

    std::vector<ContactSurface<AutoDiffXd>> surfaces;
    std::vector<PenetrationAsPointPair<AutoDiffXd>> point_pairs;
    engine_ad->ComputeContactSurfacesWithFallback(
        HydroelasticContactRepresentation::kTriangle, X_WGs_ad, &surfaces,
        &point_pairs);
    EXPECT_EQ(surfaces.size(), 0);
    EXPECT_EQ(point_pairs.size(), 1);
  }

  // Case: Rigid sphere and mesh, mutually collision filtered, with AutoDiffXd
  // -- result should be neither contact surface nor contact pair.
  {
    ProximityEngine<double> engine_d;
    const auto X_WGs_d =
        PopulateEngine(&engine_d, sphere, false, kRigid, sphere, false, kRigid);
    const auto engine_ad = engine_d.ToScalarType<AutoDiffXd>();
    unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
    GeometrySet geom_set;
    std::vector<GeometryId> ids;
    for (const auto& [id, X_WG_d] : X_WGs_d) {
      X_WGs_ad[id] = RigidTransform<AutoDiffXd>(X_WG_d.GetAsMatrix34());
      geom_set.Add(id);
      ids.push_back(id);
    }
    ASSERT_EQ(ids.size(), 2);
    engine_ad->collision_filter().Apply(
        CollisionFilterDeclaration().ExcludeWithin(geom_set),
        [&ids](const GeometrySet&, CollisionFilterScope) {
          return std::unordered_set<GeometryId>(ids.begin(), ids.end());
        },
        false);
    ASSERT_FALSE(engine_ad->collision_filter().CanCollideWith(ids[0], ids[1]));

    std::vector<ContactSurface<AutoDiffXd>> surfaces;
    std::vector<PenetrationAsPointPair<AutoDiffXd>> point_pairs;
    engine_ad->ComputeContactSurfacesWithFallback(
        HydroelasticContactRepresentation::kTriangle, X_WGs_ad, &surfaces,
        &point_pairs);
    EXPECT_EQ(surfaces.size(), 0);
    EXPECT_EQ(point_pairs.size(), 0);
  }
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

// Signed distance tests -- testing data flow; not testing the value of the
// query.

// A scene with no geometry reports no witness pairs.
GTEST_TEST(ProximityEngineTests, SignedDistanceClosestPointsOnEmptyScene) {
  ProximityEngine<double> engine;
  const unordered_map<GeometryId, RigidTransformd> X_WGs;

  const auto results =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
  EXPECT_EQ(results.size(), 0);
}

// A scene with a single anchored geometry reports no distance.
GTEST_TEST(ProximityEngineTests, SignedDistanceClosestPointsSingleAnchored) {
  ProximityEngine<double> engine;

  Sphere sphere{0.5};
  const GeometryId id = GeometryId::get_new_id();
  const unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id, RigidTransformd::Identity()}};
  engine.AddAnchoredGeometry(sphere, X_WGs.at(id), id);

  const auto results =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
  EXPECT_EQ(results.size(), 0);
}

// Tests that anchored geometry don't report closest distance with each other.
GTEST_TEST(ProximityEngineTests, SignedDistanceClosestPointsMultipleAnchored) {
  ProximityEngine<double> engine;
  unordered_map<GeometryId, RigidTransformd> X_WGs;

  const double radius = 0.5;
  Sphere sphere{radius};
  const GeometryId id_A = GeometryId::get_new_id();
  X_WGs[id_A] = RigidTransformd::Identity();
  engine.AddAnchoredGeometry(sphere, X_WGs.at(id_A), id_A);

  const GeometryId id_B = GeometryId::get_new_id();
  X_WGs[id_B] = RigidTransformd{Translation3d{1.8 * radius, 0, 0}};
  engine.AddAnchoredGeometry(sphere, X_WGs.at(id_B), id_B);

  const auto results =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
  EXPECT_EQ(results.size(), 0);
}

// Tests that the maximum distance value is respected. Confirms that two shapes
// are included/excluded based on a distance just inside and outside that
// maximum distance, respectively.
GTEST_TEST(ProximityEngineTests, SignedDistanceClosestPointsMaxDistance) {
  ProximityEngine<double> engine;
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_A, RigidTransformd::Identity()}, {id_B, RigidTransformd::Identity()}};

  const double radius = 0.5;
  Sphere sphere{radius};
  engine.AddDynamicGeometry(sphere, {}, id_A);
  engine.AddDynamicGeometry(sphere, {}, id_B);

  const double kMaxDistance = 1;
  const double kEps = 2 * std::numeric_limits<double>::epsilon();
  const double kCenterDistance = kMaxDistance + radius + radius;

  // Case: Just inside the maximum distance.
  {
    const Vector3d p_WB =
        Vector3d(2, 3, 4).normalized() * (kCenterDistance - kEps);
    X_WGs[id_B].set_translation(p_WB);
    engine.UpdateWorldPoses(X_WGs);
    const auto results =
        engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kMaxDistance);
    EXPECT_EQ(results.size(), 1);
  }

  // Case: Just outside the maximum distance.
  {
    const Vector3d p_WB =
        Vector3d(2, 3, 4).normalized() * (kCenterDistance + kEps);
    X_WGs[id_B].set_translation(p_WB);
    engine.UpdateWorldPoses(X_WGs);
    const auto results =
        engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kMaxDistance);
    EXPECT_EQ(results.size(), 0);
  }
}

// Tests the computation of signed distance for a single geometry pair. Confirms
// successful case as well as failure case.
GTEST_TEST(ProximityEngineTests, SignedDistancePairClosestPoint) {
  ProximityEngine<double> engine;
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  const GeometryId bad_id = GeometryId::get_new_id();
  unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_A, RigidTransformd::Identity()}, {id_B, RigidTransformd::Identity()}};

  const double radius = 0.5;
  Sphere sphere{radius};
  engine.AddDynamicGeometry(sphere, {}, id_A);
  engine.AddDynamicGeometry(sphere, {}, id_B);

  const double kDistance = 1;
  const double kCenterDistance = kDistance + radius + radius;
  // Displace B the desired distance in an arbitrary direction.
  const Vector3d p_WB = Vector3d(2, 3, 4).normalized() * kCenterDistance;
  X_WGs[id_B].set_translation(p_WB);
  engine.UpdateWorldPoses(X_WGs);

  // Case: good case produces the correct value.
  {
    const SignedDistancePair<double> result =
        engine.ComputeSignedDistancePairClosestPoints(id_A, id_B, X_WGs);
    EXPECT_EQ(result.id_A, id_A);
    EXPECT_EQ(result.id_B, id_B);
    EXPECT_NEAR(result.distance, kDistance,
                std::numeric_limits<double>::epsilon());
    // We're not testing *all* the fields. The callback is setting the fields,
    // we assume if ids and distance are correct, the previously tested callback
    // code does it all correctly.
  }

  // Case: the first id is invalid.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.ComputeSignedDistancePairClosestPoints(bad_id, id_B, X_WGs),
        fmt::format("The geometry given by id {} does not reference .+ used in "
                    "a signed distance query",
                    bad_id));
  }

  // Case: the second id is invalid.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.ComputeSignedDistancePairClosestPoints(id_A, bad_id, X_WGs),
        fmt::format("The geometry given by id {} does not reference .+ used in "
                    "a signed distance query",
                    bad_id));
  }

  // Case: the distance is evaluated even though the pair is filtered.
  {
    // I know the GeometrySet only has id_A and id_B, so I'll construct the
    // extracted set by hand.
    auto extract_ids = [id_A, id_B](const GeometrySet&, CollisionFilterScope) {
      return std::unordered_set<GeometryId>{id_A, id_B};
    };
    engine.collision_filter().Apply(
        CollisionFilterDeclaration().ExcludeWithin(GeometrySet{id_A, id_B}),
        extract_ids, false /* is_invariant */);
    EXPECT_NO_THROW(
        engine.ComputeSignedDistancePairClosestPoints(id_A, id_B, X_WGs));
  }
}

// ComputeSignedDistanceToPoint tests

// Test the broad-phase part of ComputeSignedDistanceToPoint.

// Confirms that non-positve thresholds produce the right value. Creates two
// penetrating spheres: A & B. The query point is *inside* the intersection of
// A and B.  We confirm that query tolerance of 0, returns two results and that
// a tolerance of penetration depth + epsilon likewise returns two results, and
// depth - epsilon omits everything.
GTEST_TEST(ProximityEngineTests, SignedDistanceToPointNonPositiveThreshold) {
  const double kRadius = 0.5;
  const double kPenetration = kRadius * 0.1;

  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  const unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_A,
       RigidTransformd{Translation3d{-kRadius + 0.5 * kPenetration, 0, 0}}},
      {id_B,
       RigidTransformd{Translation3d{kRadius - 0.5 * kPenetration, 0, 0}}}};

  Sphere sphere{kRadius};

  for (bool flip_order : {true, false}) {
    ProximityEngine<double> engine;
    // Confirm the results are sorted, regardless of the order the spheres are
    // added to the engine.
    const GeometryId first_id = flip_order ? id_B : id_A;
    const GeometryId second_id = flip_order ? id_A : id_B;
    engine.AddDynamicGeometry(sphere, {}, first_id);
    engine.AddDynamicGeometry(sphere, {}, second_id);
    engine.UpdateWorldPoses(X_WGs);

    const Vector3d p_WQ{0, 0, 0};
    std::vector<SignedDistanceToPoint<double>> results_all =
        engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, kInf);
    ASSERT_EQ(results_all.size(), 2u);
    // Make sure the result is sorted.
    auto parameters_in_order = [](const SignedDistanceToPoint<double>& p1,
                                  const SignedDistanceToPoint<double>& p2) {
      return p1.id_G < p2.id_G;
    };
    EXPECT_TRUE(parameters_in_order(results_all[0], results_all[1]));

    std::vector<SignedDistanceToPoint<double>> results_zero =
        engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, 0);
    ASSERT_EQ(results_zero.size(), 2u);
    EXPECT_TRUE(parameters_in_order(results_zero[0], results_zero[1]));

    const double kEps = std::numeric_limits<double>::epsilon();

    std::vector<SignedDistanceToPoint<double>> results_barely_in =
        engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs,
                                            -kPenetration * 0.5 + kEps);
    ASSERT_EQ(results_barely_in.size(), 2u);
    EXPECT_TRUE(
        parameters_in_order(results_barely_in[0], results_barely_in[1]));

    std::vector<SignedDistanceToPoint<double>> results_barely_out =
        engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs,
                                            -kPenetration * 0.5 - kEps);
    EXPECT_EQ(results_barely_out.size(), 0u);
  }
}

// ProximityEngine::ComputeSignedDistanceGeometryToPoint() does no math. It is
// simply responsible for acquiring the indicated geometry (if possible,
// throwing if not), bundling it up with the query point, forwarding it to the
// callback, and returning the measured result. We'll be testing that
// functionality.
GTEST_TEST(ProximityEngineTests, SignedDistanceGeometryToPoint) {
  const double kRadius = 0.5;

  const GeometryId dynamic_id = GeometryId::get_new_id();
  const GeometryId anchored_id = GeometryId::get_new_id();
  DRAKE_DEMAND(dynamic_id < anchored_id);
  const GeometryId bad_id = GeometryId::get_new_id();
  // Two different arbitrary poses: one for dynamic, one for anchored.
  const RigidTransformd X_WD(Vector3d(-10, -11, -12));
  const RigidTransformd X_WA(Vector3d(-9, -8, 7));
  const unordered_map<GeometryId, RigidTransformd> X_WGs{{dynamic_id, X_WD},
                                                         {anchored_id, X_WA}};

  Sphere sphere{kRadius};

  std::unordered_set<GeometryId> ids{anchored_id, dynamic_id};

  ProximityEngine<double> engine;

  engine.AddAnchoredGeometry(sphere, X_WA, anchored_id);
  engine.AddDynamicGeometry(sphere, X_WD, dynamic_id);
  engine.UpdateWorldPoses(X_WGs);

  // Point in arbitrary point away from the origin.
  const Vector3d p_WQ{1, 2, 3};

  const std::vector<SignedDistanceToPoint<double>> result =
      engine.ComputeSignedDistanceGeometryToPoint(p_WQ, X_WGs, ids);

  // Confirm ordering.
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0].id_G, dynamic_id);
  EXPECT_EQ(result[1].id_G, anchored_id);
  // Confirm distance.
  EXPECT_DOUBLE_EQ(result[0].distance,
                   (X_WD.translation() - p_WQ).norm() - kRadius);
  EXPECT_DOUBLE_EQ(result[1].distance,
                   (X_WA.translation() - p_WQ).norm() - kRadius);

  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.ComputeSignedDistanceGeometryToPoint(p_WQ, X_WGs, {bad_id}),
      ".*does not reference a geometry.*signed distance query");
}

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
  unordered_map<GeometryId, RigidTransformd> X_WGs;
  const double radius = 0.1;
  const Vector3d center1(1, 1, 1);
  const Vector3d center2(-1, -1, -1);
  for (const Vector3d& p_WG : {center1, center2}) {
    const RigidTransformd X_WG(Translation3d{p_WG});
    const GeometryId id = GeometryId::get_new_id();
    X_WGs[id] = X_WG;
    engine.AddAnchoredGeometry(Sphere(radius), X_WG, id);
  }
  const Vector3d p_WQ(3, 3, 3);
  // This small threshold allows no sphere.
  double threshold = 0.001;
  auto results = engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);
  EXPECT_EQ(0, results.size());

  // This threshold touches the corner of the bounding box of the first sphere.
  // It is still too small to yield any result.
  threshold = (p_WQ - (center1 + Vector3d(radius, radius, radius))).norm();
  results = engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);
  EXPECT_EQ(0, results.size());

  // This threshold barely touches outside the first sphere, so it still gives
  // no result.
  threshold = (p_WQ - center1).norm() - radius - 1e-10;
  results = engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);
  EXPECT_EQ(0, results.size());

  // This threshold barely touches inside the first sphere, so it gives
  // one result.
  threshold = (p_WQ - center1).norm() - radius + 1e-10;
  results = engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);
  EXPECT_EQ(1, results.size());

  // This threshold touches the corner of the bounding box of the second
  // sphere, so it is still too small to allow the second sphere.
  threshold = (p_WQ - (center2 + Vector3d(radius, radius, radius))).norm();
  results = engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);
  EXPECT_EQ(1, results.size());

  // This threshold barely touches the outside the second sphere, so it still
  // gives one result.
  threshold = (p_WQ - center2).norm() - radius - 1e-10;
  results = engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);
  EXPECT_EQ(1, results.size());

  // This threshold barely touches inside the second sphere, so it starts to
  // give two results.
  threshold = (p_WQ - center2).norm() - radius + 1e-10;
  results = engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);
  EXPECT_EQ(2, results.size());
}

// Penetration tests -- testing data flow; not testing the value of the query.

// A scene with no geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationOnEmptyScene) {
  ProximityEngine<double> engine;

  auto results = engine.ComputePointPairPenetration({});
  EXPECT_EQ(results.size(), 0);
}

// A scene with a single anchored geometry reports no penetrations.
GTEST_TEST(ProximityEngineTests, PenetrationSingleAnchored) {
  ProximityEngine<double> engine;

  Sphere sphere{0.5};
  RigidTransformd pose = RigidTransformd::Identity();
  const GeometryId id = GeometryId::get_new_id();
  engine.AddAnchoredGeometry(sphere, pose, id);
  auto results = engine.ComputePointPairPenetration({{id, pose}});
  EXPECT_EQ(results.size(), 0);
}

// Tests that anchored geometry aren't collided against each other -- even if
// they actually *are* in penetration.
GTEST_TEST(ProximityEngineTests, PenetrationMultipleAnchored) {
  ProximityEngine<double> engine;

  const double radius = 0.5;
  Sphere sphere{radius};
  const GeometryId id1 = GeometryId::get_new_id();
  std::unordered_map<GeometryId, RigidTransformd> X_WGs;
  X_WGs.emplace(id1, RigidTransformd::Identity());
  const GeometryId id2 = GeometryId::get_new_id();
  RigidTransformd pose = RigidTransformd::Identity();
  engine.AddAnchoredGeometry(sphere, pose, GeometryId::get_new_id());
  pose.set_translation({1.8 * radius, 0, 0});
  X_WGs.emplace(id2, pose);
  engine.AddAnchoredGeometry(sphere, pose, GeometryId::get_new_id());
  auto results = engine.ComputePointPairPenetration(X_WGs);
  EXPECT_EQ(results.size(), 0);
}

// Utility test for evaluating the following "ResultOrdering" test. It produces
// a "ring" of spheres such that each sphere makes contact with its two
// neighbors. So, N spheres produce N contacts. Given the input radius and
// count (N), produces a map of geometry ids to poses for the N spheres.
unordered_map<GeometryId, RigidTransformd> MakeCollidingRing(double radius,
                                                             int N) {
  /*
              y
              │                   Example of N = 4.
              o
             ╱│╲
           ╱  │  ╲ s
         ╱    │θ   ╲
  ──────o───────────o───── x
         ╲    │    ╱
           ╲  │  ╱
             ╲│╱
              o
              │
  Put spheres centered at the dots indicated. We want the distance between two
  "adjacent" spheres to be s = 1.9 * r (so that they collide). So, how far from
  the center should those centers be? We can use the equilateral triangle and
  law of sines. Each triangle has angles θ = π/2, α = π/4, α = π/4. The distance
  from the origin d can be found by solving for:

       d            s
   --------  =  --------  --> d = s / sin(θ) * sin(α)
    sin(α)       sin(θ)

  Note: as long as we define θ = 2π / N, this will work for positioning N
  spheres.

  We play weird games with the geometry ids. If we simply enumerate the ids
  in *order* (let's call them A, B, C, D) as we walk around the ring, then we'll
  have contact pairs (A, B), (B, C), (C, D), (D, A)
  */
  DRAKE_DEMAND(radius > 0);
  DRAKE_DEMAND(N > 0);
  unordered_map<GeometryId, RigidTransformd> poses;
  const double span = 2 * M_PI / N;
  const double d = 1.9 * radius / sin(span) * sin((M_PI - span) / 2);
  for (int i = 0; i < N; ++i) {
    const double angle = i * span;
    const Vector3d p_WSo = Vector3d(cos(angle), sin(angle), 0) * d;
    poses[GeometryId::get_new_id()] = RigidTransformd(p_WSo);
  }
  return poses;
}

// Confirms that the ComputePointPairPenetration() computation returns the
// same results twice in a row. This test is explicitly required because it is
// known that updating the pose in the FCL tree can lead to erratic ordering.
GTEST_TEST(ProximityEngineTests, PenetrationAsPointPairResultOrdering) {
  ProximityEngine<double> engine;

  const double r = 0.5;
  // For radius = 0.5, we find 4 spheres is sufficient to expose the old bug.
  unordered_map<GeometryId, RigidTransformd> poses = MakeCollidingRing(r, 4);

  const Sphere sphere{r};
  for (const auto& pair : poses) {
    engine.AddDynamicGeometry(sphere, {}, pair.first);
  }
  engine.UpdateWorldPoses(poses);
  const auto results1 = engine.ComputePointPairPenetration(poses);
  ASSERT_EQ(results1.size(), poses.size());

  engine.UpdateWorldPoses(poses);
  const auto results2 = engine.ComputePointPairPenetration(poses);
  ASSERT_EQ(results1.size(), poses.size());

  for (size_t i = 0; i < poses.size(); ++i) {
    EXPECT_EQ(results1[i].id_A, results2[i].id_A);
    EXPECT_EQ(results1[i].id_B, results2[i].id_B);
  }
}

// Confirms that the FindCollisionCandidates() computation returns the
// same results twice in a row. This test is explicitly required because it is
// known that updating the pose in the FCL tree can lead to erratic ordering.
GTEST_TEST(ProximityEngineTests, FindCollisionCandidatesResultOrdering) {
  ProximityEngine<double> engine;

  const double r = 0.5;
  // For radius = 0.5, we find 4 spheres is sufficient to expose the old bug.
  unordered_map<GeometryId, RigidTransformd> poses = MakeCollidingRing(r, 4);

  const Sphere sphere{r};
  for (const auto& pair : poses) {
    engine.AddDynamicGeometry(sphere, {}, pair.first);
  }
  engine.UpdateWorldPoses(poses);
  const auto results1 = engine.FindCollisionCandidates();
  ASSERT_EQ(results1.size(), poses.size());

  engine.UpdateWorldPoses(poses);
  const auto results2 = engine.FindCollisionCandidates();
  ASSERT_EQ(results1.size(), poses.size());

  for (size_t i = 0; i < poses.size(); ++i) {
    EXPECT_EQ(results1[i].first(), results2[i].first());
    EXPECT_EQ(results1[i].second(), results2[i].second());
  }
}

// Confirms that the ComputeContactSurfaces() computation returns the
// same results twice in a row. This test is explicitly required because it is
// known that updating the pose in the FCL tree can lead to erratic ordering.
// This logic doesn't depend on mesh representation, so we test it with a single
// representation.
class ProximityEngineHydro : public testing::Test {
 protected:
  void SetUp() override {
    const double r = 0.5;
    // For radius = 0.5, we find 4 spheres is sufficient to expose the old bug.
    // And, for contact surfaces, we need an even number to guarantee rigid-soft
    // alternation.
    poses_ = MakeCollidingRing(r, 4);

    ProximityProperties soft_properties;
    AddCompliantHydroelasticProperties(r, 1e8, &soft_properties);
    ProximityProperties rigid_properties;
    AddRigidHydroelasticProperties(r, &rigid_properties);

    // Extract the ids from poses and sort them so that we know we're assigning
    // appropriate alternativing compliance.
    std::vector<GeometryId> ids;
    for (const auto& pair : poses_) {
      ids.push_back(pair.first);
    }
    std::sort(ids.begin(), ids.end());

    int n = 0;
    const Sphere sphere{r};
    for (const auto& id : ids) {
      engine_.AddDynamicGeometry(sphere, {}, id,
                                 (n % 2) ? soft_properties : rigid_properties);
      ++n;
    }
  }

  ProximityEngine<double> engine_;
  unordered_map<GeometryId, RigidTransformd> poses_;
};

TEST_F(ProximityEngineHydro, ComputeContactSurfacesResultOrdering) {
  engine_.UpdateWorldPoses(poses_);
  const auto results1 = engine_.ComputeContactSurfaces(
      HydroelasticContactRepresentation::kTriangle, poses_);
  ASSERT_EQ(results1.size(), poses_.size());

  engine_.UpdateWorldPoses(poses_);
  const auto results2 = engine_.ComputeContactSurfaces(
      HydroelasticContactRepresentation::kTriangle, poses_);
  ASSERT_EQ(results2.size(), poses_.size());

  for (size_t i = 0; i < poses_.size(); ++i) {
    EXPECT_EQ(results1[i].id_M(), results2[i].id_M());
    EXPECT_EQ(results1[i].id_N(), results2[i].id_N());
  }
}

// Confirms that the ComputeContactSurfacesWithFallback() computation returns
// the same results twice in a row. This test is explicitly required because it
// is known that updating the pose in the FCL tree can lead to erratic ordering.
// This logic is independent of mesh representation, so, we only test for one
// mesh type.
class ProximityEngineHydroWithFallback : public testing::Test {
 protected:
  void SetUp() override {
    const double r = 0.5;
    // For this case we want at least four contacts *of each type*. So, we order
    // geometries as: S S R R S S R R. This will give us four soft contacts and
    // four rigid contacts.
    N_ = 8;
    poses_ = MakeCollidingRing(r, N_);

    ProximityProperties soft_properties;
    AddCompliantHydroelasticProperties(r / 2, 1e8, &soft_properties);
    ProximityProperties rigid_properties;
    AddRigidHydroelasticProperties(r, &rigid_properties);

    // Extract the ids from poses and sort them so that we know we're assigning
    // appropriate alternativing compliance.
    std::vector<GeometryId> ids;
    for (const auto& pair : poses_) {
      ids.push_back(pair.first);
    }
    std::sort(ids.begin(), ids.end());

    int n = 0;
    const Sphere sphere{r};
    for (const auto& id : ids) {
      bool is_soft = (n % 4) < 2;
      engine_.AddDynamicGeometry(sphere, {}, id,
                                 is_soft ? soft_properties : rigid_properties);
      ++n;
    }
  }

  size_t N_;
  ProximityEngine<double> engine_;
  unordered_map<GeometryId, RigidTransformd> poses_;
};

TEST_F(ProximityEngineHydroWithFallback,
       ComputeContactSurfacesWithFallbackResultOrdering) {
  engine_.UpdateWorldPoses(poses_);
  vector<ContactSurface<double>> surfaces1;
  vector<PenetrationAsPointPair<double>> points1;
  engine_.ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation::kTriangle, poses_, &surfaces1,
      &points1);
  // The arrangement (see MakeCollidingRing()) looks somewhat
  // like this (R = rigid sphere, C = compliant sphere):
  //
  //      R  R
  //    C      C
  //    C      C
  //      R  R
  //
  // Only two R-R (rigid-rigid) contacts are point contacts.
  const size_t num_point_contacts = 2;
  // The remaining contacts are either R-C (rigid-compliant) or C-C
  // (compliant-compliant) hydroelastic contact patches.
  const size_t num_patch_contacts = N_ - 2;
  ASSERT_EQ(surfaces1.size(), num_patch_contacts);
  ASSERT_EQ(points1.size(), num_point_contacts);

  engine_.UpdateWorldPoses(poses_);
  vector<ContactSurface<double>> surfaces2;
  vector<PenetrationAsPointPair<double>> points2;
  engine_.ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation::kTriangle, poses_, &surfaces2,
      &points2);
  ASSERT_EQ(surfaces2.size(), num_patch_contacts);
  ASSERT_EQ(points2.size(), num_point_contacts);

  for (size_t i = 0; i < num_patch_contacts; ++i) {
    EXPECT_EQ(surfaces1[i].id_M(), surfaces2[i].id_M());
    EXPECT_EQ(surfaces1[i].id_N(), surfaces2[i].id_N());
  }
  for (size_t i = 0; i < num_point_contacts; ++i) {
    EXPECT_EQ(points1[i].id_A, points2[i].id_A);
    EXPECT_EQ(points1[i].id_B, points2[i].id_B);
  }
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
  // position. The sphere is indicated by its `id` which belongs to the given
  // `source_id`. If `is_colliding` is true, the sphere is placed in a colliding
  // configuration.
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

  // Updates a pose in X_WGs_ to be colliding or non-colliding. Then updates the
  // position of all dynamic geometries.
  void MoveDynamicSphere(GeometryId id, bool is_colliding,
                         ProximityEngine<double>* engine = nullptr) {
    engine = engine == nullptr ? &engine_ : engine;

    DRAKE_DEMAND(engine->num_geometries() == static_cast<int>(X_WGs_.size()));

    const double x_pos = is_colliding ? colliding_x_ : free_x_;
    X_WGs_[id].set_translation({x_pos, 0, 0});

    engine->UpdateWorldPoses(X_WGs_);
  }

  // Poses have been defined as doubles; get them in the required scalar type.
  template <typename T>
  unordered_map<GeometryId, RigidTransform<T>> GetTypedPoses() const {
    unordered_map<GeometryId, RigidTransform<T>> typed_X_WG;
    for (const auto& id_pose_pair : X_WGs_) {
      const GeometryId id = id_pose_pair.first;
      const RigidTransformd& X_WG = id_pose_pair.second;
      typed_X_WG[id] = X_WG.cast<T>();
    }
    return typed_X_WG;
  }

  // Compute penetration and confirm that a single penetration with the expected
  // properties was found. Provide the geometry ids of the sphere located at
  // the origin and the sphere positioned to be in collision.
  template <typename T>
  void ExpectPenetration(GeometryId origin_sphere, GeometryId colliding_sphere,
                         ProximityEngine<T>* engine) {
    std::vector<PenetrationAsPointPair<T>> penetration_results =
        engine->ComputePointPairPenetration(GetTypedPoses<T>());
    ASSERT_EQ(penetration_results.size(), 1);
    const PenetrationAsPointPair<T>& penetration = penetration_results[0];

    std::vector<SignedDistancePair<T>> distance_results =
        engine->ComputeSignedDistancePairwiseClosestPoints(GetTypedPoses<T>(),
                                                           kInf);
    ASSERT_EQ(distance_results.size(), 1);
    const SignedDistancePair<T>& distance = distance_results[0];

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
    SignedDistancePair<T> expected_distance;
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
    std::vector<PenetrationAsPointPair<T>> penetration_results =
        engine->ComputePointPairPenetration(GetTypedPoses<T>());
    EXPECT_EQ(penetration_results.size(), 0);

    std::vector<SignedDistancePair<T>> distance_results =
        engine->ComputeSignedDistancePairwiseClosestPoints(GetTypedPoses<T>(),
                                                           kInf);
    ASSERT_EQ(distance_results.size(), 0);
  }

  // Compute penetration and confirm that none were found.
  template <typename T>
  void ExpectNoPenetration(GeometryId origin_sphere,
                           GeometryId colliding_sphere,
                           ProximityEngine<T>* engine) {
    std::vector<PenetrationAsPointPair<double>> penetration_results =
        engine->ComputePointPairPenetration(GetTypedPoses<T>());
    EXPECT_EQ(penetration_results.size(), 0);

    std::vector<SignedDistancePair<double>> distance_results =
        engine->ComputeSignedDistancePairwiseClosestPoints(X_WGs_, kInf);
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
  unordered_map<GeometryId, RigidTransformd> X_WGs_;
  const double radius_{0.5};
  const Sphere sphere_{radius_};
  const double free_x_{2.5 * radius_};
  const double colliding_x_{1.5 * radius_};
};

// Tests collision between dynamic and anchored sphere. One case colliding, one
// case *not* colliding.
TEST_F(SimplePenetrationTest, PenetrationDynamicAndAnchored) {
  // Set up anchored geometry.
  RigidTransformd pose = RigidTransformd::Identity();
  const GeometryId anchored_id = GeometryId::get_new_id();
  engine_.AddAnchoredGeometry(sphere_, pose, anchored_id);

  // Set up dynamic geometry.
  const GeometryId dynamic_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, dynamic_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  X_WGs_[anchored_id] = pose;
  X_WGs_[dynamic_id] = RigidTransformd::Identity();

  // Non-colliding case.
  MoveDynamicSphere(dynamic_id, false /* not colliding */);
  ExpectNoPenetration(anchored_id, dynamic_id, &engine_);

  // Colliding case.
  MoveDynamicSphere(dynamic_id, true /* colliding */);
  ExpectPenetration(anchored_id, dynamic_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(anchored_id, dynamic_id, &copy_engine);

  // Test scalar-converted engines.
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToScalarType<AutoDiffXd>();
  ExpectPenetration(anchored_id, dynamic_id, ad_engine.get());
  std::unique_ptr<ProximityEngine<Expression>> sym_engine =
      engine_.ToScalarType<Expression>();
  DRAKE_EXPECT_THROWS_MESSAGE(
      ExpectPenetration(anchored_id, dynamic_id, sym_engine.get()),
      ".*are not supported for scalar type drake::symbolic::Expression.*");
}

// Performs the same collision test between two dynamic spheres which belong to
// the same source.
TEST_F(SimplePenetrationTest, PenetrationDynamicAndDynamicSingleSource) {
  const GeometryId origin_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, origin_id);

  GeometryId collide_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, collide_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  X_WGs_[origin_id] = RigidTransformd::Identity();
  X_WGs_[collide_id] = RigidTransformd::Identity();

  // Non-colliding case.
  MoveDynamicSphere(collide_id, false /* not colliding */);
  ExpectNoPenetration(origin_id, collide_id, &engine_);

  // Colliding case.
  MoveDynamicSphere(collide_id, true /* colliding */);
  ExpectPenetration(origin_id, collide_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectPenetration(origin_id, collide_id, &copy_engine);

  // Test scalar-converted engines.
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToScalarType<AutoDiffXd>();
  ExpectPenetration(origin_id, collide_id, ad_engine.get());
  std::unique_ptr<ProximityEngine<Expression>> sym_engine =
      engine_.ToScalarType<Expression>();
  DRAKE_EXPECT_THROWS_MESSAGE(
      ExpectPenetration(origin_id, collide_id, sym_engine.get()),
      ".*are not supported for scalar type drake::symbolic::Expression.*");
}

// Tests if collisions exist between dynamic and anchored sphere. One case
// colliding, one case *not* colliding.
TEST_F(SimplePenetrationTest, HasCollisionsDynamicAndAnchored) {
  // Set up anchored geometry.
  const RigidTransformd pose = RigidTransformd::Identity();
  const GeometryId anchored_id = GeometryId::get_new_id();
  engine_.AddAnchoredGeometry(sphere_, pose, anchored_id);

  // Set up dynamic geometry.
  const GeometryId dynamic_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, dynamic_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  X_WGs_[anchored_id] = pose;
  X_WGs_[dynamic_id] = RigidTransformd::Identity();

  // Non-colliding case.
  MoveDynamicSphere(dynamic_id, false /* not colliding */);
  EXPECT_FALSE(engine_.HasCollisions());

  // Colliding case.
  MoveDynamicSphere(dynamic_id, true /* colliding */);
  EXPECT_TRUE(engine_.HasCollisions());

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  EXPECT_TRUE(copy_engine.HasCollisions());

  // Test scalar-converted engines.
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToScalarType<AutoDiffXd>();
  EXPECT_TRUE(ad_engine->HasCollisions());
  std::unique_ptr<ProximityEngine<Expression>> sym_engine =
      engine_.ToScalarType<Expression>();
  EXPECT_TRUE(sym_engine->HasCollisions());
}

// Performs the same collision test between two dynamic spheres which belong to
// the same source.
TEST_F(SimplePenetrationTest, HasCollisionsDynamicAndDynamicSingleSource) {
  const GeometryId origin_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, origin_id);

  GeometryId collide_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, collide_id);
  EXPECT_EQ(engine_.num_geometries(), 2);

  X_WGs_[origin_id] = RigidTransformd::Identity();
  X_WGs_[collide_id] = RigidTransformd::Identity();

  // Non-colliding case.
  MoveDynamicSphere(collide_id, false /* not colliding */);
  EXPECT_FALSE(engine_.HasCollisions());

  // Colliding case.
  MoveDynamicSphere(collide_id, true /* colliding */);
  EXPECT_TRUE(engine_.HasCollisions());

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  EXPECT_TRUE(copy_engine.HasCollisions());

  // Test scalar-converted engines.
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToScalarType<AutoDiffXd>();
  EXPECT_TRUE(ad_engine->HasCollisions());
  std::unique_ptr<ProximityEngine<Expression>> sym_engine =
      engine_.ToScalarType<Expression>();
  EXPECT_TRUE(sym_engine->HasCollisions());
}

// There was a bug (23406). If anchored geometries existed but they were not
// in collision with dynamic geometries, collisions between dynamics geometries
// would be ignored and the HasCollisions() query would return a lie. This is
// a test against that bug coming back.
TEST_F(SimplePenetrationTest, HasCollisionsAnchoredInterference) {
  const GeometryId dynamic_id_1 = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, dynamic_id_1);

  const GeometryId dynamic_id_2 = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, dynamic_id_2);
  EXPECT_EQ(engine_.num_geometries(), 2);

  X_WGs_[dynamic_id_1] = RigidTransformd::Identity();
  X_WGs_[dynamic_id_2] = RigidTransformd::Identity();

  EXPECT_TRUE(engine_.HasCollisions());

  // The anchored geometry must be non-colliding, but close enough that its
  // bounding box overlaps.
  const Vector3d p_WA =
      Vector3d(1, 1, 0).normalized() * (sphere_.radius() * 2.1);
  const GeometryId anchored_id = GeometryId::get_new_id();
  engine_.AddAnchoredGeometry(sphere_, RigidTransformd(p_WA), anchored_id);
  EXPECT_EQ(engine_.num_geometries(), 3);
  // We still have collisions (original code would report false).
  EXPECT_TRUE(engine_.HasCollisions());
}

// Performs the same collision test where the geometries have been filtered.
TEST_F(SimplePenetrationTest, WithCollisionFilters) {
  GeometryId origin_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, origin_id);

  GeometryId collide_id = GeometryId::get_new_id();
  engine_.AddDynamicGeometry(sphere_, {}, collide_id);
  ASSERT_EQ(engine_.num_geometries(), 2);

  X_WGs_[origin_id] = RigidTransformd::Identity();
  X_WGs_[collide_id] = RigidTransformd::Identity();

  ASSERT_TRUE(engine_.collision_filter().CanCollideWith(origin_id, collide_id));

  // I know the GeometrySet only has id_A and id_B, so I'll construct the
  // extracted set by hand.
  auto extract_ids = [origin_id, collide_id](const GeometrySet&,
                                             CollisionFilterScope) {
    return std::unordered_set<GeometryId>{origin_id, collide_id};
  };
  engine_.collision_filter().Apply(CollisionFilterDeclaration().ExcludeWithin(
                                       GeometrySet{origin_id, collide_id}),
                                   extract_ids, false /* is_invariant */);

  EXPECT_FALSE(
      engine_.collision_filter().CanCollideWith(origin_id, collide_id));

  // Non-colliding case.
  MoveDynamicSphere(collide_id, false /* not colliding */);
  ExpectIgnoredPenetration(origin_id, collide_id, &engine_);

  // Colliding case.
  MoveDynamicSphere(collide_id, true /* colliding */);
  ExpectIgnoredPenetration(origin_id, collide_id, &engine_);

  // Test colliding case on copy.
  ProximityEngine<double> copy_engine(engine_);
  ExpectIgnoredPenetration(origin_id, collide_id, &copy_engine);

  // Test scalar-converted engines.
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ad_engine =
      engine_.ToScalarType<AutoDiffXd>();
  ExpectIgnoredPenetration(origin_id, collide_id, ad_engine.get());
  std::unique_ptr<ProximityEngine<Expression>> sym_engine =
      engine_.ToScalarType<Expression>();
  ExpectIgnoredPenetration(origin_id, collide_id, sym_engine.get());
}

// Confirms that non-positive thresholds produce the right value. Creates three
// spheres: A, B, & C. A is separated from B & C and B & C are penetrating.
// We confirm that query tolerance of 0, returns B & C and that a tolerance
// of penetration depth + epsilon returns B & C, and depth - epsilon omits
// everything.
GTEST_TEST(ProximityEngineTests, PairwiseSignedDistanceNonPositiveThreshold) {
  ProximityEngine<double> engine;
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();
  const GeometryId id3 = GeometryId::get_new_id();
  const double kRadius = 0.5;
  const unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id1, RigidTransformd{Translation3d{0, 2 * kRadius, 0}}},
      {id2, RigidTransformd{Translation3d{-kRadius * 0.9, 0, 0}}},
      {id3, RigidTransformd{Translation3d{kRadius * 0.9, 0, 0}}}};

  Sphere sphere{kRadius};
  engine.AddDynamicGeometry(sphere, {}, id1);
  engine.AddDynamicGeometry(sphere, {}, id2);
  engine.AddDynamicGeometry(sphere, {}, id3);
  engine.UpdateWorldPoses(X_WGs);
  std::vector<SignedDistancePair<double>> results_all =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
  ASSERT_EQ(results_all.size(), 3u);
  // Make sure the result is sorted
  auto parameters_in_order = [](const SignedDistancePair<double>& p1,
                                const SignedDistancePair<double>& p2) {
    if (p1.id_A != p2.id_A) return p1.id_A < p2.id_A;
    return p1.id_B < p2.id_B;
  };
  EXPECT_TRUE(parameters_in_order(results_all[0], results_all[1]));
  EXPECT_TRUE(parameters_in_order(results_all[1], results_all[2]));

  std::vector<SignedDistancePair<double>> results_zero =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, 0);
  EXPECT_EQ(results_zero.size(), 1u);

  const double penetration = -kRadius * 0.2;
  const double kEps = std::numeric_limits<double>::epsilon();

  std::vector<SignedDistancePair<double>> results_barely_in =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs,
                                                        penetration + kEps);
  EXPECT_EQ(results_barely_in.size(), 1u);

  std::vector<SignedDistancePair<double>> results_barely_out =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs,
                                                        penetration - kEps);
  EXPECT_EQ(results_barely_out.size(), 0u);
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
  const double w = radius;  // Box width smaller than diameter.
  const double half_w = w / 2;
  const double h = 10 * radius;  // Box height much larger than sphere.
  const double d = 10 * radius;  // Box depth much larger than sphere.
  const double eps = std::numeric_limits<double>::epsilon();
  const GeometryId box_id = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Box{w, h, d}, {}, box_id);
  const GeometryId sphere_id = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Sphere{radius}, {}, sphere_id);

  unordered_map<GeometryId, RigidTransformd> poses{
      {box_id, RigidTransformd::Identity()},
      {sphere_id, RigidTransformd::Identity()}};
  // clang-format off
  std::vector<SpherePunchData> test_data{
      // In non-penetration, contact_normal and depth values don't matter; they
      // are not tested.
      {"non-penetration",
       {radius + half_w + 0.1, 0.0, 0.0}, 0, {0.0, 0.0, 0.0}, -1.0},
      {"shallow penetration -- sphere center outside of box",
       {radius + 0.75 * half_w, 0.0, 0.0}, 1, {-1.0, 0.0, 0.0}, 0.25 * half_w},
      {"deep penetration -- sphere contacts opposite side of the box",
       {radius - half_w, 0.0, 0.0}, 1, {-1.0, 0.0, 0.0}, w},
      {"sphere's origin is just to the right of the box center",
       {eps, 0.0, 0.0}, 1, {-1.0, 0.0, 0.0}, radius + half_w - eps},
      {"sphere's center has crossed the box's origin - flipped normal",
       {-eps, 0.0, 0.0}, 1, {1.0, 0.0, 0.0}, radius + half_w - eps}};
  // clang-format on
  for (const auto& test : test_data) {
    poses[sphere_id].set_translation(test.sphere_pose);
    engine.UpdateWorldPoses(poses);
    std::vector<PenetrationAsPointPair<double>> results =
        engine.ComputePointPairPenetration(poses);

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

// TODO(SeanCurtis-TRI): Remove these geometric tests from here and put them
//  in their own test. We're keeping them, ultimately, so that we can use these
//  tests for our own implementation of shape-shape point-intersection
//  algorithms. In the short-term, these tests should be expressed directly
//  in terms of the penetration_as_point_pair::Callback and moved to that set
//  of tests.

// Robust Box-Primitive tests. Tests collision of the box with other primitives
// in a uniform framework. These tests parallel tests located in fcl.
//
// All of the tests below here are using the callback to exercise the black box.
// They exist because of FCL; FCL's unit tests were sporadic at best and these
// tests revealed errors/properties of FCL that weren't otherwise apparent.
// Ultimately, these tests don't belong here. But they can be re-used when we
// replace FCL with Drake's own implementations.
//
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
//       plane and then displace downward (0, 0, -radius). Described as "prone".
//   - capsule: There are two valid configurations (radius & length >> depth).
//     - Place a "standing" capsule at (0, 0, -length/2 - radius).
//     - Rotate the capsule so that its length axis is parallel with the z = 0
//       plane and then displace downward (0, 0, -radius). Described as "prone".
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
    TangentConvex,
    TangentStandingCapsule,
    TangentProneCapsule
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
    const GeometryId tangent_id = GeometryId::get_new_id();
    engine_.AddDynamicGeometry(shape(shape_type), {}, tangent_id);

    const GeometryId box_id = GeometryId::get_new_id();
    engine_.AddDynamicGeometry(box_, {}, box_id);

    // Confirm that there are no other geometries interfering.
    ASSERT_EQ(engine_.num_dynamic(), 2);

    // Update the poses of the geometry.
    unordered_map<GeometryId, RigidTransformd> poses{
        {tangent_id, shape_pose(shape_type)}, {box_id, X_WB}};
    engine_.UpdateWorldPoses(poses);
    std::vector<PenetrationAsPointPair<double>> results =
        engine_.ComputePointPairPenetration(poses);

    ASSERT_EQ(results.size(), 1u)
        << "Against tangent " << shape_name(shape_type);

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
      GTEST_FAIL() << fmt::format(
          "Wrong geometry ids reported in contact for tangent {}. Expected {} "
          "and {}. Got {} and {}",
          shape_name(shape_type), tangent_id, box_id, contact.id_A,
          contact.id_B);
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
    const math::RotationMatrixd R_WB(
        AngleAxisd(std::atan(M_SQRT2), Vector3d(M_SQRT1_2, -M_SQRT1_2, 0)));

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
      case TangentStandingCapsule:
        return "standing capsule";
      case TangentProneCapsule:
        return "prone capsule";
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
      case TangentStandingCapsule:
      case TangentProneCapsule:
        return tangent_capsule_;
    }
    // GCC considers this function ill-formed - no apparent return value. This
    // exception alleviates its concern.
    throw std::logic_error(
        "Trying to acquire shape for unknown shape enumerated value: " +
        std::to_string(shape));
  }

  // Map enumeration to tangent pose.
  RigidTransformd shape_pose(TangentShape shape) {
    RigidTransformd pose = RigidTransformd::Identity();
    switch (shape) {
      case TangentPlane:
        break;  // leave it at the identity
      case TangentSphere:
        pose.set_translation({0, 0, -kRadius});
        break;
      case TangentBox:
      // The tangent convex is a cube of the same size as the tangent box.
      // That is why we give them the same pose.
      case TangentConvex:
      case TangentStandingCylinder:
        pose.set_translation({0, 0, -kLength / 2});
        break;
      case TangentStandingCapsule:
        pose.set_translation({0, 0, -kLength / 2 - kRadius});
        break;
      case TangentProneCylinder:
      case TangentProneCapsule:
        pose = RigidTransformd(AngleAxisd{M_PI_2, Vector3d::UnitX()},
                               Vector3d{0, 0, -kRadius});
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

  // The various geometries used in the collision test.
  const Box box_{1, 1, 1};
  const Sphere tangent_sphere_{kRadius};
  const Box tangent_box_{kLength, kLength, kLength};
  const HalfSpace tangent_plane_;  // Default construct the z = 0 plane.
  const Cylinder tangent_cylinder_{kRadius, kLength};
  // We scale the convex shape by 5.0 to match the tangent_box_ of size 10.0.
  // The file "quad_cube.obj" contains the cube of size 2.0.
  const Convex tangent_convex_{
      drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj"), 5.0};
  const Capsule tangent_capsule_{kRadius, kLength};

  const Vector3d p_WC_{0, 0, -kDepth};
  const Vector3d p_BoC_B_{-0.5, -0.5, -0.5};
};

// See documentation. All geometry constants must be >= kDepth * 100.
const double BoxPenetrationTest::kDepth = 1e-3;
const double BoxPenetrationTest::kRadius = 1;
const double BoxPenetrationTest::kLength = 10;

TEST_F(BoxPenetrationTest, TangentPlane1) {
  TestCollision1(TangentPlane, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentPlane2) {
  TestCollision2(TangentPlane, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentBox1) {
  TestCollision1(TangentBox, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentBox2) {
  TestCollision2(TangentBox, 1e-12);
}

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

TEST_F(BoxPenetrationTest, TangentStandingCapsule1) {
  // TODO(tehbelinda): We should check why we cannot use a smaller tolerance.
  TestCollision1(TangentStandingCapsule, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentStandingCapsule2) {
  TestCollision2(TangentStandingCapsule, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentProneCapsule1) {
  // TODO(tehbelinda): We should check why we cannot use a smaller tolerance.
  TestCollision1(TangentProneCapsule, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentProneCapsule2) {
  // TODO(tehbelinda): We should check why we cannot use a smaller tolerance.
  TestCollision2(TangentProneCapsule, 1e-4);
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
  engine.AddDynamicGeometry(Box(0.49, 0.63, 0.015), {}, id_A);
  engine.AddDynamicGeometry(Cylinder(0.08, 0.002), {}, id_B);

  // Original translation was p_WA = (-0.145, -0.63, 0.2425) and
  // p_WB = (0, -0.6, 0.251), respectively.
  RigidTransformd X_WA(Eigen::AngleAxisd{M_PI_2, Vector3d::UnitZ()},
                       Vector3d{-0.25, 0, 0});
  RigidTransformd X_WB(Vector3d{0, 0, 0.0085});
  const unordered_map<GeometryId, RigidTransformd> X_WG{{id_A, X_WA},
                                                        {id_B, X_WB}};
  engine.UpdateWorldPoses(X_WG);
  std::vector<GeometryId> geometry_map{id_A, id_B};
  auto pairs = engine.ComputePointPairPenetration(X_WG);
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
  engine.AddDynamicGeometry(Sphere(0.5), {}, id_D);

  RigidTransformd X_WA{Translation3d{-3, 0, 0}};
  engine.AddAnchoredGeometry(Sphere(0.5), X_WA, id_A);

  RigidTransformd X_WD{Translation3d{-3, 0.75, 0}};

  std::unordered_map<GeometryId, math::RigidTransform<double>> X_WGs{
      {id_A, X_WA}, {id_D, X_WD}};
  engine.UpdateWorldPoses(X_WGs);
  auto pairs = engine.ComputePointPairPenetration(X_WGs);

  EXPECT_EQ(pairs.size(), 1);

  // Confirm that it survives copying.
  ProximityEngine<double> engine_copy(engine);
  engine_copy.UpdateWorldPoses({{id_D, X_WD}});
  auto pairs_copy = engine_copy.ComputePointPairPenetration(X_WGs);
  EXPECT_EQ(pairs_copy.size(), 1);
}

// Basic smoke test for the autodiffibility of the signed distance computation.
// Tests against the anchored geometry. Specifically, it confirms that while
// poses are set with double, the calculation is done with AutoDiff and
// derivatives come through.
GTEST_TEST(ProximityEngineTests, ComputePointSignedDistanceAutoDiffAnchored) {
  ProximityEngine<AutoDiffXd> engine;

  const double kEps = std::numeric_limits<double>::epsilon();

  // Given a sphere with radius 0.7, centered on p_WSo, the point p_SQ = (2,3,6)
  // is outside G at the positive distance 6.3.
  const double expected_distance = 6.3;
  const Vector3d p_SQ_W{2, 3, 6};
  const Vector3d p_WS_W{0.5, 1.25, -2};
  const Vector3d p_WQ{p_SQ_W + p_WS_W};
  Vector3<AutoDiffXd> p_WQ_ad = math::InitializeAutoDiff(p_WQ);

  // An empty world inherently produces no results.
  {
    const unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs;
    const auto results = engine.ComputeSignedDistanceToPoint(p_WQ_ad, X_WGs);
    EXPECT_EQ(results.size(), 0);
  }

  // Against an anchored sphere.
  {
    const RigidTransformd X_WS = RigidTransformd(p_WS_W);
    const GeometryId anchored_id = GeometryId::get_new_id();
    engine.AddAnchoredGeometry(Sphere(0.7), X_WS, anchored_id);
    const RigidTransform<AutoDiffXd> X_WS_ad = X_WS.cast<AutoDiffXd>();
    const unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs{
        {anchored_id, X_WS_ad}};

    // Distance is just beyond the threshold.
    {
      std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
          engine.ComputeSignedDistanceToPoint(p_WQ_ad, X_WGs,
                                              expected_distance - 1e-14);
      EXPECT_EQ(results.size(), 0);
    }

    // Distance is just within the threshold
    {
      std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
          engine.ComputeSignedDistanceToPoint(p_WQ_ad, X_WGs,
                                              expected_distance + 1e-14);
      EXPECT_EQ(results.size(), 1);
      const SignedDistanceToPoint<AutoDiffXd>& distance_data = results[0];
      // The autodiff seems to lose a couple of bits relative to the known
      // answer.
      EXPECT_NEAR(distance_data.distance.value(), expected_distance, 4 * kEps);
      // The analytical `grad_W` value should match the autodiff-computed
      // gradient.
      const Vector3d ddistance_dp_WQ = distance_data.distance.derivatives();
      const Vector3d grad_W = math::ExtractValue(distance_data.grad_W);
      EXPECT_TRUE(CompareMatrices(ddistance_dp_WQ, grad_W, kEps));
    }
  }
}

// Basic smoke test for the autodiffibility of the signed distance computation.
// Tests against the dynamic geometry. Specifically, it confirms that while
// poses are set with double, the calculation is done with AutoDiff and
// derivatives come through.
GTEST_TEST(ProximityEngineTests, ComputePointSignedDistanceAutoDiffDynamic) {
  ProximityEngine<AutoDiffXd> engine;

  const double kEps = std::numeric_limits<double>::epsilon();

  // Given a sphere with radius 0.7, centered on p_WSo, the point p_SQ = (2,3,6)
  // is outside G at the positive distance 6.3.
  const double expected_distance = 6.3;
  const Vector3d p_SQ{2, 3, 6};
  const Vector3d p_WS{0.5, 1.25, -2};
  const Vector3d p_WQ{p_SQ + p_WS};
  Vector3<AutoDiffXd> p_WQ_ad = math::InitializeAutoDiff(p_WQ);

  // Against a dynamic sphere.
  const GeometryId id = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Sphere(0.7), {}, id);
  const auto X_WS_ad = RigidTransformd(p_WS).cast<AutoDiffXd>();
  const unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs{
      {id, X_WS_ad}};
  engine.UpdateWorldPoses(X_WGs);

  // Distance is just beyond the threshold.
  {
    std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
        engine.ComputeSignedDistanceToPoint(p_WQ_ad, X_WGs,
                                            expected_distance - 1e-14);
    EXPECT_EQ(results.size(), 0);
  }

  // Distance is just within the threshold
  {
    std::vector<SignedDistanceToPoint<AutoDiffXd>> results =
        engine.ComputeSignedDistanceToPoint(p_WQ_ad, X_WGs,
                                            expected_distance + 1e-14);
    EXPECT_EQ(results.size(), 1);
    const SignedDistanceToPoint<AutoDiffXd>& distance_data = results[0];
    // The autodiff seems to lose a couple of bits relative to the known
    // answer.
    EXPECT_NEAR(distance_data.distance.value(), expected_distance, 4 * kEps);
    // The analytical `grad_W` value should match the autodiff-computed
    // gradient.
    const Vector3d ddistance_dp_WQ = distance_data.distance.derivatives();
    const Vector3d grad_W = math::ExtractValue(distance_data.grad_W);
    EXPECT_TRUE(CompareMatrices(ddistance_dp_WQ, grad_W, kEps));
  }
}

GTEST_TEST(ProximityEngineTests, ComputePairwiseSignedDistanceAutoDiff) {
  ProximityEngine<AutoDiffXd> engine;

  const double kEps = std::numeric_limits<double>::epsilon();

  const double radius = 0.7;

  // Two spheres with radius 0.7 whose centers are 7 m apart. The literal
  // values just keep it from being trivial zero-able.
  const double expected_distance = 7 - radius - radius;
  const Vector3d p_SQ{2, 3, 6};
  const Vector3d p_WS{0.5, 1.25, -2};
  const Vector3d p_WQ{p_SQ + p_WS};
  Vector3<AutoDiffXd> p_WQ_ad = math::InitializeAutoDiff(p_WQ);

  // Add a pair of dynamic spheres. We'll differentiate w.r.t. the pose of the
  // first sphere.
  const GeometryId id1 = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Sphere(radius), {}, id1);
  const RigidTransform<AutoDiffXd> X_WS1_ad(p_WQ_ad);

  const GeometryId id2 = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Sphere(radius), {}, id2);
  const auto X_WS2_ad = RigidTransformd(p_WS).cast<AutoDiffXd>();

  const unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs{
      {id1, X_WS1_ad}, {id2, X_WS2_ad}};
  engine.UpdateWorldPoses(X_WGs);

  std::vector<SignedDistancePair<AutoDiffXd>> results =
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
  ASSERT_EQ(results.size(), 1);

  const SignedDistancePair<AutoDiffXd>& distance_data = results[0];
  // The autodiff seems to lose a couple of bits relative to the known
  // answer.
  EXPECT_NEAR(distance_data.distance.value(), expected_distance, 4 * kEps);
  // The hand-computed `grad_W` value should match the autodiff-computed
  // gradient.
  const Vector3d ddistance_dp_WQ = distance_data.distance.derivatives();
  const Vector3d grad_w = math::ExtractValue(distance_data.nhat_BA_W);
  EXPECT_TRUE(CompareMatrices(ddistance_dp_WQ, grad_w, kEps));
}

// Tests that an unsupported geometry causes the engine to throw.
GTEST_TEST(ProximityEngineTests,
           ComputePairwiseSignedDistanceAutoDiffUnsupported) {
  ProximityEngine<AutoDiffXd> engine;

  // Add two geometries that can't be queried.
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Box(1, 2, 3), {}, id1);
  engine.AddDynamicGeometry(Box(2, 4, 6), {}, id2);

  const unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs{
      {id1, RigidTransform<AutoDiffXd>::Identity()},
      {id2, RigidTransform<AutoDiffXd>::Identity()}};
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf),
      "Signed distance queries between shapes 'Box' and 'Box' are not "
      "supported for scalar type drake::AutoDiffXd.*");
}

// Tests that an unsupported geometry causes the engine to throw.
GTEST_TEST(ProximityEngineTests, ExpressionUnsupported) {
  ProximityEngine<Expression> engine;

  // Add two geometries that can't be queried.
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Box(1, 2, 3), {}, id1);
  engine.AddDynamicGeometry(Box(2, 4, 6), {}, id2);

  const unordered_map<GeometryId, RigidTransform<Expression>> X_WGs{
      {id1, RigidTransform<Expression>::Identity()},
      {id2, RigidTransform<Expression>::Identity()}};
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.ComputePointPairPenetration(X_WGs),
      "Penetration queries between shapes 'Box' and 'Box' are not supported "
      "for scalar type drake::symbolic::Expression.*");
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

GTEST_TEST(ProximityEngineTest, ThrowForStrictRejectedContacts) {
  using enum HydroelasticType;
  using enum HydroelasticContactRepresentation;
  const bool anchored{true};
  const Sphere sphere{0.2};
  const HalfSpace half_space;

  // Confirms that if the intersecting pair is missing hydroelastic
  // representation that an exception is thrown. This test applies *no*
  // collision filters to guarantee that the body of the calculator gets
  // exercised in all cases.
  {
    ProximityEngine<double> engine;
    const auto X_WGs = PopulateEngine(&engine, sphere, anchored, kRigid, sphere,
                                      !anchored, kUndefined);

    // We test only a single "underrepresented" configuration (rigid,
    // undefined) because we rely on the tests on MaybeCalcContactSurface() to
    // have explored all the ways that the kUnsupported calculation result is
    // returned. This configuration is representative of that set.
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.ComputeContactSurfaces(kTriangle, X_WGs),
        "Requested a contact surface between a pair of geometries without "
        "hydroelastic representation .+ rigid .+ undefined .+");
  }

  // Confirms that if the intersecting pair is rigid-rigid, an exception is
  // thrown. This test applies *no* collision filters to guarantee that the
  // body of the calculator gets exercised in all cases.
  {
    ProximityEngine<double> engine;
    const auto X_WGs = PopulateEngine(&engine, sphere, anchored, kRigid, sphere,
                                      !anchored, kRigid);

    // We test only a single "same-compliance" configuration (rigid, rigid)
    // because we rely on the tests on MaybeMakeContactSurface() to have
    // explored all the ways that the calculation result is returned. This
    // configuration is representative of that set.
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.ComputeContactSurfaces(kTriangle, X_WGs),
        "Requested contact between two rigid objects .+");
  }

  // Confirms that if the intersecting pair consists of two half spaces that an
  // exception is thrown.
  {
    ProximityEngine<double> engine;
    // They must have different compliance types in order to get past the same
    // compliance type condition.
    const auto X_WGs = PopulateEngine(&engine, half_space, anchored, kRigid,
                                      half_space, !anchored, kCompliant);

    DRAKE_EXPECT_THROWS_MESSAGE(engine.ComputeContactSurfaces(kTriangle, X_WGs),
                                "Requested contact between two half spaces .+");
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
