#include "drake/geometry/proximity/hydroelastic_internal.h"

#include <cmath>
#include <functional>
#include <limits>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {
namespace {

using Eigen::Vector3d;
using std::function;
using std::make_unique;
using std::pow;

GTEST_TEST(SoftMeshTest, TestCopyMoveAssignConstruct) {
  const Sphere sphere(0.5);
  const double resolution_hint = 0.5;
  auto mesh = make_unique<VolumeMesh<double>>(MakeSphereVolumeMesh<double>(
      sphere, resolution_hint, TessellationStrategy::kSingleInteriorVertex));
  const double hydroelastic_modulus = 1e+7;
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeSpherePressureField(sphere, mesh.get(), hydroelastic_modulus));

  const SoftMesh original(std::move(mesh), std::move(pressure));

  // Test copy-assignment operator.
  {
    SoftMesh copy;
    copy = original;

    // Test for uniqueness.
    EXPECT_NE(&original.mesh(), &copy.mesh());
    EXPECT_NE(&original.pressure(), &copy.pressure());
    EXPECT_NE(&original.bvh(), &copy.bvh());

    EXPECT_TRUE(copy.mesh().Equal(original.mesh()));

    const auto& copy_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            copy.pressure());
    const auto& original_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            original.pressure());
    EXPECT_TRUE(copy_pressure.Equal(original_pressure));

    EXPECT_TRUE(copy.bvh().Equal(original.bvh()));
  }

  // Test copy constructor.
  {
    SoftMesh copy(original);

    // Test for uniqueness.
    EXPECT_NE(&original.mesh(), &copy.mesh());
    EXPECT_NE(&original.pressure(), &copy.pressure());
    EXPECT_NE(&original.bvh(), &copy.bvh());

    EXPECT_TRUE(copy.mesh().Equal(original.mesh()));

    const auto& copy_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            copy.pressure());
    const auto& original_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            original.pressure());
    EXPECT_TRUE(copy_pressure.Equal(original_pressure));

    EXPECT_TRUE(copy.bvh().Equal(original.bvh()));
  }

  // Test move constructor and move-assignment operator.
  // We will move the content from `start` to `move_constructed` to
  // `move_assigned`, each time confirming that the target of the move has taken
  // ownership.
  {
    SoftMesh start(original);  // Assume the copy constructor is correct.

    // Grab raw pointers so we can determine that their ownership changes due to
    // move semantics.
    const VolumeMesh<double>* const mesh_ptr = &start.mesh();
    const VolumeMeshFieldLinear<double, double>* const pressure_ptr =
        &start.pressure();
    const Bvh<Obb, VolumeMesh<double>>* const bvh_ptr = &start.bvh();

    // Test move constructor.
    SoftMesh move_constructed(std::move(start));
    EXPECT_EQ(&move_constructed.mesh(), mesh_ptr);
    EXPECT_EQ(&move_constructed.pressure(), pressure_ptr);
    EXPECT_EQ(&move_constructed.bvh(), bvh_ptr);

    // Test move-assignment operator.
    SoftMesh move_assigned;
    move_assigned = std::move(move_constructed);
    EXPECT_EQ(&move_assigned.mesh(), mesh_ptr);
    EXPECT_EQ(&move_assigned.pressure(), pressure_ptr);
    EXPECT_EQ(&move_assigned.bvh(), bvh_ptr);
  }
}

// SoftGeometry can represent either a mesh or a half space (and in the future,
// possibly more types). Therefore, in construction, the source can be one of
// any of the types and in assignment, the target can likewise be any
// supported type. We do not explicitly test all combinations. We rely on the
// fact that SoftGeometry's management of these exclusive types is handled by
// std::variant and the move/copy semantics of the underlying data types
// (already tested). If SoftGeometry changes its implementation details, this
// logic would need to be revisited.
GTEST_TEST(SoftGeometryTest, TestCopyMoveAssignConstruct) {
  const Sphere sphere(0.5);
  const double resolution_hint = 0.5;
  auto mesh = make_unique<VolumeMesh<double>>(MakeSphereVolumeMesh<double>(
      sphere, resolution_hint, TessellationStrategy::kSingleInteriorVertex));
  const double hydroelastic_modulus = 1e+7;
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeSpherePressureField(sphere, mesh.get(), hydroelastic_modulus));

  const SoftGeometry original(SoftMesh(std::move(mesh), std::move(pressure)));

  // Test copy-assignment operator.
  {
    // Initialize `dut` as a SoftGeometry representing a half space.
    // Then, change it to a SoftGeometry representing a mesh by copy-assignment.
    SoftGeometry dut(SoftHalfSpace{1e+7});
    dut = original;

    // Test for uniqueness. The contents have different memory addresses.
    EXPECT_NE(&original.mesh(), &dut.mesh());
    EXPECT_NE(&original.pressure_field(), &dut.pressure_field());
    EXPECT_NE(&original.bvh(), &dut.bvh());

    EXPECT_TRUE(dut.mesh().Equal(original.mesh()));
    const auto& copy_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            dut.pressure_field());
    const auto& original_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            original.pressure_field());
    EXPECT_TRUE(copy_pressure.Equal(original_pressure));
    EXPECT_TRUE(dut.bvh().Equal(original.bvh()));
  }

  // Test copy constructor.
  {
    SoftGeometry copy(original);

    // Test for uniqueness. The contents have different memory addresses.
    EXPECT_NE(&original.mesh(), &copy.mesh());
    EXPECT_NE(&original.pressure_field(), &copy.pressure_field());
    EXPECT_NE(&original.bvh(), &copy.bvh());

    EXPECT_TRUE(copy.mesh().Equal(original.mesh()));
    const auto& copy_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            copy.pressure_field());
    const auto& original_pressure =
        static_cast<const VolumeMeshFieldLinear<double, double>&>(
            original.pressure_field());
    EXPECT_TRUE(copy_pressure.Equal(original_pressure));
    EXPECT_TRUE(copy.bvh().Equal(original.bvh()));
  }

  // Test move constructor and move-assignment operator.
  // We will move the content from `start` to `move_constructed` to
  // `move_assigned`, each time confirming that the target of the move has taken
  // ownership.
  {
    SoftGeometry start(original);  // Assume the copy constructor is correct.

    // Grab raw pointers so we can determine that their ownership changes due to
    // move semantics.
    const VolumeMesh<double>* const mesh_ptr = &start.mesh();
    const VolumeMeshFieldLinear<double, double>* const pressure_ptr =
        &start.pressure_field();
    const Bvh<Obb, VolumeMesh<double>>* const bvh_ptr = &start.bvh();

    // Test move constructor.
    SoftGeometry move_constructed(std::move(start));
    EXPECT_EQ(&move_constructed.mesh(), mesh_ptr);
    EXPECT_EQ(&move_constructed.pressure_field(), pressure_ptr);
    EXPECT_EQ(&move_constructed.bvh(), bvh_ptr);

    // Test move-assignment operator.
    // Initialize `move_assigned` as a SoftGeometry representing a half spce.
    // Then, change it to a SoftGeometry of a soft mesh by move-assignment
    // from `move_constructed`.
    SoftGeometry move_assigned(SoftHalfSpace{1e+7});
    move_assigned = std::move(move_constructed);
    EXPECT_EQ(&move_assigned.mesh(), mesh_ptr);
    EXPECT_EQ(&move_assigned.pressure_field(), pressure_ptr);
    EXPECT_EQ(&move_assigned.bvh(), bvh_ptr);
  }
}

GTEST_TEST(RigidMeshTest, TestCopyMoveAssignConstruct) {
  const Sphere sphere(0.5);
  const double resolution_hint = 0.5;
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeSphereSurfaceMesh<double>(sphere, resolution_hint));

  const RigidMesh original(std::move(mesh));

  // Test copy-assignment operator.
  {
    RigidMesh copy;
    copy = original;

    // Test for uniqueness.
    EXPECT_NE(&original.mesh(), &copy.mesh());
    EXPECT_NE(&original.bvh(), &copy.bvh());

    EXPECT_TRUE(copy.mesh().Equal(original.mesh()));
    EXPECT_TRUE(copy.bvh().Equal(original.bvh()));
  }

  // Test copy constructor.
  {
    RigidMesh copy(original);

    // Test for uniqueness.
    EXPECT_NE(&original.mesh(), &copy.mesh());
    EXPECT_NE(&original.bvh(), &copy.bvh());

    EXPECT_TRUE(copy.mesh().Equal(original.mesh()));
    EXPECT_TRUE(copy.bvh().Equal(original.bvh()));
  }

  // Test move constructor and move-assignment operator.
  // We will move the content from `start` to `move_constructed` to
  // `move_assigned`, each time confirming that the target of the move has taken
  // ownership.
  {
    RigidMesh start(original);  // Assume the copy constructor is correct.

    // Grab raw pointers so we can determine that their ownership changes due to
    // move semantics.
    const TriangleSurfaceMesh<double>* const mesh_ptr = &start.mesh();
    const Bvh<Obb, TriangleSurfaceMesh<double>>* const bvh_ptr = &start.bvh();

    // Test move constructor.
    RigidMesh move_constructed(std::move(start));
    EXPECT_EQ(&move_constructed.mesh(), mesh_ptr);
    EXPECT_EQ(&move_constructed.bvh(), bvh_ptr);

    // Test move-assignment operator.
    RigidMesh move_assigned;
    move_assigned = std::move(move_constructed);
    EXPECT_EQ(&move_assigned.mesh(), mesh_ptr);
    EXPECT_EQ(&move_assigned.bvh(), bvh_ptr);
  }
}

// RigidGeometry can represent either a mesh or a half space (and in the future,
// possibly more types). Therefore, in construction, the source can be one of
// any of the types and in assignment, the target can likewise be any
// supported type. We do not explicitly test all combinations. We rely on the
// fact that RigidGeometry's management of these exclusive types is handled by
// std::optional and the move/copy semantics of the underlying data types
// (already tested). If RigidGeometry changes its implementation details, this
// logic would need to be revisited.
GTEST_TEST(RigidGeometryTest, TestCopyMoveAssignConstruct) {
  const RigidGeometry original(
      RigidMesh(make_unique<TriangleSurfaceMesh<double>>(
          MakeSphereSurfaceMesh<double>(Sphere(1.25), 2.0))));

  // Test copy-assignment operator.
  {
    // Initialize `dut` as a RigidGeometry representing a half space. Then,
    // change it to a RigidGeometry representing a mesh by copy-assignment.
    RigidGeometry dut(HalfSpace{});
    dut = original;

    // Test for uniqueness. The contents have different memory addresses.
    EXPECT_NE(&original.mesh(), &dut.mesh());
    EXPECT_NE(&original.bvh(), &dut.bvh());

    EXPECT_TRUE(dut.mesh().Equal(original.mesh()));
    EXPECT_TRUE(dut.bvh().Equal(original.bvh()));
  }

  // Test copy constructor.
  {
    RigidGeometry copy(original);

    // Test for uniqueness. Their contents are at different memory addresses.
    EXPECT_NE(&original.mesh(), &copy.mesh());
    EXPECT_NE(&original.bvh(), &copy.bvh());

    EXPECT_TRUE(copy.mesh().Equal(original.mesh()));
    EXPECT_TRUE(copy.bvh().Equal(original.bvh()));
  }

  // Test move constructor and move-assignment operator.
  // We will move the content from `start` to `move_constructed` to
  // `move_assigned`, each time confirming that the target of the move has taken
  // ownership.
  {
    RigidGeometry start(original);  // Assume the copy constructor is correct.

    // Grab raw pointers so we can determine that their ownership changes due to
    // move semantics.
    const TriangleSurfaceMesh<double>* const mesh_ptr = &start.mesh();
    const Bvh<Obb, TriangleSurfaceMesh<double>>* const bvh_ptr = &start.bvh();

    // Test move constructor.
    RigidGeometry move_constructed(std::move(start));
    EXPECT_EQ(&move_constructed.mesh(), mesh_ptr);
    EXPECT_EQ(&move_constructed.bvh(), bvh_ptr);

    // Test move-assignment operator.
    // Initialize `move_assigned` as a RigidGeometry representing a half space.
    // Then, change it to a RigidGeometry of a mesh by move-assignment.
    RigidGeometry move_assigned(HalfSpace{});
    move_assigned = std::move(move_constructed);
    EXPECT_EQ(&move_assigned.mesh(), mesh_ptr);
    EXPECT_EQ(&move_assigned.bvh(), bvh_ptr);
  }
}

// Tests the simple public API of the hydroelastic::Geometries: adding
// geometries and querying the data stored.
GTEST_TEST(Hydroelastic, GeometriesPopulationAndQuery) {
  Geometries geometries;

  // Ids that haven't been added report as undefined.
  GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties;
  AddRigidHydroelasticProperties(1.0, &rigid_properties);

  GeometryId soft_id = GeometryId::get_new_id();
  ProximityProperties soft_properties;
  AddSoftHydroelasticProperties(1.0, 1e8, &soft_properties);

  GeometryId bad_id = GeometryId::get_new_id();
  EXPECT_EQ(geometries.hydroelastic_type(rigid_id),
            HydroelasticType::kUndefined);
  EXPECT_EQ(geometries.hydroelastic_type(soft_id),
            HydroelasticType::kUndefined);
  EXPECT_EQ(geometries.hydroelastic_type(bad_id), HydroelasticType::kUndefined);

  // Once added, they report the appropriate type.
  geometries.MaybeAddGeometry(Sphere(0.5), soft_id, soft_properties);
  EXPECT_EQ(geometries.hydroelastic_type(soft_id),
            HydroelasticType::kSoft);
  geometries.MaybeAddGeometry(Sphere(0.5), rigid_id, rigid_properties);
  EXPECT_EQ(geometries.hydroelastic_type(rigid_id), HydroelasticType::kRigid);

  // Ids that report the correct type, successfully access the appropriate
  // representation.
  DRAKE_EXPECT_NO_THROW(geometries.soft_geometry(soft_id));
  DRAKE_EXPECT_NO_THROW(geometries.rigid_geometry(rigid_id));
}

GTEST_TEST(Hydroelastic, RemoveGeometry) {
  Geometries geometries;

  // Add a rigid geometry.
  const GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties;
  AddRigidHydroelasticProperties(1.0, &rigid_properties);
  geometries.MaybeAddGeometry(Sphere(0.5), rigid_id, rigid_properties);
  ASSERT_EQ(geometries.hydroelastic_type(rigid_id), HydroelasticType::kRigid);

  // Add a soft geometry.
  const GeometryId soft_id = GeometryId::get_new_id();
  ProximityProperties soft_properties;
  AddSoftHydroelasticProperties(1.0, 1e8, &soft_properties);
  geometries.MaybeAddGeometry(Sphere(0.5), soft_id, soft_properties);
  ASSERT_EQ(geometries.hydroelastic_type(soft_id), HydroelasticType::kSoft);

  // Case 1: Remove a geometry that has no representation is a no-op.
  const GeometryId bad_id = GeometryId::get_new_id();
  ASSERT_EQ(geometries.hydroelastic_type(bad_id), HydroelasticType::kUndefined);
  DRAKE_EXPECT_NO_THROW(geometries.RemoveGeometry(bad_id));
  ASSERT_EQ(geometries.hydroelastic_type(bad_id), HydroelasticType::kUndefined);
  ASSERT_EQ(geometries.hydroelastic_type(rigid_id), HydroelasticType::kRigid);
  ASSERT_EQ(geometries.hydroelastic_type(soft_id), HydroelasticType::kSoft);

  // Case 2: Removing a rigid geometry has no effect on anything else. We copy
  // the geometries to make sure that there is always something else to remain
  // untouched.
  {
    Geometries copy{geometries};
    DRAKE_EXPECT_NO_THROW(copy.RemoveGeometry(rigid_id));
    ASSERT_EQ(copy.hydroelastic_type(rigid_id), HydroelasticType::kUndefined);
    ASSERT_EQ(copy.hydroelastic_type(soft_id), HydroelasticType::kSoft);
  }

  // Case 3: Removing a soft geometry has no effect on anything else. We copy
  // the geometries to make sure that there is always something else to remain
  // untouched.
  {
    Geometries copy{geometries};
    DRAKE_EXPECT_NO_THROW(copy.RemoveGeometry(soft_id));
    ASSERT_EQ(copy.hydroelastic_type(soft_id), HydroelasticType::kUndefined);
    ASSERT_EQ(copy.hydroelastic_type(rigid_id), HydroelasticType::kRigid);
  }
}

class HydroelasticRigidGeometryTest : public ::testing::Test {
 protected:
  /* Creates a simple set of properties for generating rigid geometry. */
  ProximityProperties rigid_properties(double edge_length = 0.1) const {
    ProximityProperties properties;
    AddRigidHydroelasticProperties(edge_length, &properties);
    return properties;
  }
};

// TODO(SeanCurtis-TRI): As new shape specifications are added, they are
//  implicitly unsupported and should be added here (and in
//  UnsupportedSofthapes).
// Smoke test for shapes that are *known* to be unsupported as rigid objects.
// NOTE: This will spew warnings to the log.
TEST_F(HydroelasticRigidGeometryTest, UnsupportedRigidShapes) {
}

// Confirm support for a rigid half space. Tests that a hydroelastic
// representation is made, and samples the representation to look for evidence
// of it being the *right* representation.
TEST_F(HydroelasticRigidGeometryTest, HalfSpace) {
  ProximityProperties props = rigid_properties();

  std::optional<RigidGeometry> half_space =
      MakeRigidRepresentation(HalfSpace(), props);
  ASSERT_NE(half_space, std::nullopt);
  EXPECT_TRUE(half_space->is_half_space());

  DRAKE_EXPECT_THROWS_MESSAGE(
      half_space->mesh(), std::runtime_error,
      "RigidGeometry::mesh.* cannot be invoked .* half space");
  DRAKE_EXPECT_THROWS_MESSAGE(
      half_space->bvh(), std::runtime_error,
      "RigidGeometry::bvh.* cannot be invoked .* half space");
}

// Confirm support for a rigid Sphere. Tests that a hydroelastic representation
// is made, and samples the representation to look for evidence of it being the
// *right* representation.
TEST_F(HydroelasticRigidGeometryTest, Sphere) {
  const double radius = 0.5;
  Sphere sphere_spec(radius);

  ProximityProperties props = rigid_properties(0.5);
  std::optional<RigidGeometry> sphere =
      MakeRigidRepresentation(sphere_spec, props);
  ASSERT_NE(sphere, std::nullopt);
  ASSERT_FALSE(sphere->is_half_space());

  const TriangleSurfaceMesh<double>& mesh = sphere->mesh();
  for (int v = 0; v < mesh.num_vertices(); ++v) {
    ASSERT_NEAR(mesh.vertex(v).norm(), radius, 1e-15);
  }
}

// Confirm support for a rigid Box. Tests that a hydroelastic representation
// is made, and samples the representation to look for evidence of it being the
// *right* representation.
TEST_F(HydroelasticRigidGeometryTest, Box) {
  const double edge_len = 0.5;
  // Pick a characteristic length *larger* than the box dimensions so that
  // I only get 8 vertices - one at each corner. This is merely evidence that
  // the box method is called -- we rely on tests of that functionality to
  // create more elaborate meshes with smaller edge lengths.
  ProximityProperties props = rigid_properties(1.5 * edge_len);

  std::optional<RigidGeometry> box =
      MakeRigidRepresentation(Box(edge_len, edge_len, edge_len), props);
  ASSERT_NE(box, std::nullopt);
  ASSERT_FALSE(box->is_half_space());

  const TriangleSurfaceMesh<double>& mesh = box->mesh();
  EXPECT_EQ(mesh.num_vertices(), 8);
  // Because it is a cube centered at the origin, the distance from the origin
  // to each vertex should be sqrt(3) * edge_len / 2.
  const double expected_dist = std::sqrt(3) * edge_len / 2;
  for (int v = 0; v < mesh.num_vertices(); ++v) {
    ASSERT_NEAR(mesh.vertex(v).norm(), expected_dist, 1e-15);
  }
}

template <typename T>
std::array<T, 3> unpack(Vector3<T> x) { return {x(0), x(1), x(2) }; }

// Confirm support for a rigid Cylinder. Tests that a hydroelastic
// representation is made, and samples the representation to look for
// evidence of it being the *right* representation.
TEST_F(HydroelasticRigidGeometryTest, Cylinder) {
  const double radius = 1.0;
  const double length = 2.0;
  // Pick a characteristic length *larger* than the cylinder dimensions to
  // get the coarsest mesh. This is merely evidence that the mesh generator is
  // called -- we rely on tests of that functionality to create more
  // elaborate meshes with smaller edge lengths.
  ProximityProperties props = rigid_properties(1.5 * length);

  std::optional<RigidGeometry> cylinder =
      MakeRigidRepresentation(Cylinder(radius, length), props);
  ASSERT_NE(cylinder, std::nullopt);
  ASSERT_FALSE(cylinder->is_half_space());

  // Smoke test the surface mesh.
  const TriangleSurfaceMesh<double>& mesh = cylinder->mesh();
  EXPECT_EQ(mesh.num_vertices(), 8);
  EXPECT_EQ(mesh.num_triangles(), 12);
  for (int v = 0; v < mesh.num_vertices(); ++v) {
    const auto [x, y, z] = unpack(mesh.vertex(v));
    // Only check that the vertex is within the cylinder. It does not check
    // that the vertex is near the surface of the cylinder.  We rely on the
    // correctness of the mesh generator.
    ASSERT_LE(pow(x, 2) + pow(y, 2), pow(radius, 2) + 1e-15);
    ASSERT_LE(pow(z, 2), pow(length / 2, 2) + 1e-15);
  }
}

// Confirm support for a rigid Capsule. Tests that a hydroelastic
// representation is made, and verifies that all vertices ot the representation
// lie on the surface of the capsule.
TEST_F(HydroelasticRigidGeometryTest, Capsule) {
  const double radius = 1.0;
  const double length = 2.0;
  // Pick a characteristic length *larger* than the capsule dimensions to
  // get the coarsest mesh. This is merely evidence that the mesh generator is
  // called -- we rely on tests of that functionality to create more
  // elaborate meshes with smaller edge lengths.
  ProximityProperties props = rigid_properties(1.5 * length);

  const Capsule& capsule_shape = Capsule(radius, length);

  std::optional<RigidGeometry> capsule =
      MakeRigidRepresentation(capsule_shape, props);
  ASSERT_NE(capsule, std::nullopt);
  ASSERT_FALSE(capsule->is_half_space());

  // Smoke test the surface mesh.
  const TriangleSurfaceMesh<double>& mesh = capsule->mesh();
  EXPECT_EQ(mesh.num_vertices(), 8);
  EXPECT_EQ(mesh.num_triangles(), 12);

  for (const Vector3d& p_MV : mesh.vertices()) {
    // Check that the vertex is near the surface of the capsule.
    ASSERT_NEAR(CalcDistanceToSurface(capsule_shape, p_MV), 0.0, 1e-15);
  }

  // Create rigid representation, passing a smaller resolution hint to verify
  // that the hint is being consumed downstream.
  ProximityProperties props_fine = rigid_properties(2.0 * M_PI * radius / 6.0);
  std::optional<RigidGeometry> capsule_fine =
      MakeRigidRepresentation(capsule_shape, props_fine);

  const TriangleSurfaceMesh<double>& mesh_fine = capsule_fine->mesh();
  EXPECT_GT(mesh_fine.num_vertices(), mesh.num_vertices());
  EXPECT_GT(mesh_fine.num_elements(), mesh.num_elements());
}

// Confirm support for a rigid Ellipsoid. Tests that a hydroelastic
// representation is made, and samples the representation to look for
// evidence of it being the *right* representation.
TEST_F(HydroelasticRigidGeometryTest, Ellipsoid) {
  // Lengths of the three semi-principal axes of the ellipsoid:
  //     (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
  const double a = 0.5;
  const double b = 0.8;
  const double c = 0.3;
  // Pick a characteristic length *larger* than the ellipsoid dimensions to
  // get the coarsest mesh. This is merely evidence that the mesh generator is
  // called -- we rely on tests of that functionality to create more
  // elaborate meshes with smaller edge lengths.
  ProximityProperties props = rigid_properties(2.0 * b);

  std::optional<RigidGeometry> ellipsoid =
      MakeRigidRepresentation(Ellipsoid(a, b, c), props);
  ASSERT_NE(ellipsoid, std::nullopt);
  ASSERT_FALSE(ellipsoid->is_half_space());

  // Smoke test the surface mesh.
  const TriangleSurfaceMesh<double>& mesh = ellipsoid->mesh();
  EXPECT_EQ(mesh.num_vertices(), 6);
  EXPECT_EQ(mesh.num_triangles(), 8);
  for (int v = 0; v < mesh.num_vertices(); ++v) {
    const auto [x, y, z] = unpack(mesh.vertex(v));
    ASSERT_NEAR(pow(x / a, 2) + pow(y / b, 2) + pow(z / c, 2), 1.0, 1e-15);
  }
}

// Confirm that a mesh type (convex/mesh) has a rigid representation. We rely
// on the fact that we're loading a unit cube (vertices one unit away from the
// origin along each axis) to confirm that the correct mesh got loaded. We also
// confirm that the scale factor is included in the rigid representation.
template <typename MeshType>
void TestRigidMeshType() {
  std::string file = FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  // Empty props since its contents do not matter.
  ProximityProperties props;

  constexpr double kEps = 2 * std::numeric_limits<double>::epsilon();

  for (const double scale : {1.0, 5.1, 0.4}) {
    std::optional<RigidGeometry> geometry =
        MakeRigidRepresentation(MeshType(file, scale), props);
    ASSERT_NE(geometry, std::nullopt);
    ASSERT_FALSE(geometry->is_half_space());

    // We only check that the obj file was read by verifying the number of
    // vertices and triangles, which depend on the specific content of
    // the obj file.
    const TriangleSurfaceMesh<double>& surface_mesh = geometry->mesh();
    EXPECT_EQ(surface_mesh.num_vertices(), 8);
    EXPECT_EQ(surface_mesh.num_triangles(), 12);

    // The scale factor multiplies the measure of every vertex position, so
    // the expected distance of the vertex to the origin should be:
    // scale * sqrt(3) (because the original mesh was the unit sphere).
    const double expected_dist = std::sqrt(3) * scale;
    for (int v = 0; v < surface_mesh.num_vertices(); ++v) {
      const double dist = surface_mesh.vertex(v).norm();
      ASSERT_NEAR(dist, expected_dist, scale * kEps)
          << "for scale: " << scale << " at vertex " << v;
    }
  }
}

// Confirm support for a rigid Mesh. Tests that a hydroelastic representation
// is made.
TEST_F(HydroelasticRigidGeometryTest, Mesh) {
  SCOPED_TRACE("Rigid Mesh");
  TestRigidMeshType<Mesh>();
}

// Confirm support for a rigid Convex. Tests that a hydroelastic representation
// is made.
TEST_F(HydroelasticRigidGeometryTest, Convex) {
  SCOPED_TRACE("Rigid Convex");
  TestRigidMeshType<Convex>();
}

// Template magic to instantiate a particular kind of shape at compile time.
template <typename ShapeType>
ShapeType make_default_shape() {
  throw std::logic_error(
      "Running Hydroelastic*GeometryErrorTests for an unsupported shape "
      "type");
}

template <>
Sphere make_default_shape<Sphere>() {
  return Sphere(0.5);
}

template <>
Box make_default_shape<Box>() {
  return Box(0.5, 1.25, 3.5);
}

template <>
Cylinder make_default_shape<Cylinder>() {
  return Cylinder(0.5, 1.25);
}

template <>
Capsule make_default_shape<Capsule>() {
  return Capsule(0.5, 1.25);
}

template <>
Ellipsoid make_default_shape<Ellipsoid>() {
  return Ellipsoid(0.5, 0.8, 0.3);
}

template <>
HalfSpace make_default_shape<HalfSpace>() {
  return HalfSpace();
}

// Boilerplate for testing error conditions relating to properties. Its purpose
// is to test that the `Make*Representation` (either "Rigid" or "Soft")
// family of functions correctly validate all required properties. A property
// value can be wrong for one of three reasons:
//
//   - Missing property
//   - Wrong type for property
//   - Invalid value (maybe)
//
// This is sufficiently generic to test both kinds of geometry (rigid and soft)
// based on any shape, for a property of any type. It confirms that properly
// formatted errors are emitted in all cases.
//
// Not all properties validate invalid values. It might simply be treated as a
// black box. The third error condition is only handled if an example "bad"
// value is provided (see `bad_value` below).
//
// @param shape_spec     The shape from which we attempt to create a hydro-
//                       elastic representation.
// @param group_name     The name of the property group (in which the property
//                       lives).
// @param property_name  The name of the property (in the `group_name` group)
//                       to be tested.
// @param compliance     A string representing the compliance being requested
//                       ("rigid" or "soft").
// @param maker          The function that processes the shape and properties.
//                       Note: this is declared to return void; the
//                       Make*Representation() methods will need to be wrapped.
// @param bad_value      If provided, sets the property to this value to test
//                       validation against bad values.
// @param props          The baseline properties to start the test from --
//                       properties are generally tested in some sequence.
//                       Each test may add a property to a copy of this input
//                       to test a particular aspect.
// @tparam ShapeType  The derived class from geometry::Shape (e.g., Sphere, Box,
//                    etc.)
// @tparam ValueTYpe  The type of value under test.
template <typename ShapeType, typename ValueType>
void TestPropertyErrors(
    const ShapeType& shape_spec, const char* group_name,
    const char* property_name, const char* compliance,
    function<void(const ShapeType&, const ProximityProperties&)> maker,
    std::optional<ValueType> bad_value, const ProximityProperties& props) {
  ShapeName shape_name(shape_spec);

  // Error case: missing property value.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        maker(shape_spec, props), std::logic_error,
        fmt::format("Cannot create {} {}.+'{}'\\) property", compliance,
                    shape_name, property_name));
  }

  // Error case: property value is wrong type.
  {
    ProximityProperties wrong_value(props);
    wrong_value.AddProperty(group_name, property_name, "10");
    // This error message comes from GeometryProperties::GetProperty().
    DRAKE_EXPECT_THROWS_MESSAGE(
        maker(shape_spec, wrong_value), std::logic_error,
        fmt::format(".*The property \\('{}', '{}'\\) exists, but is of a "
                    "different type.+string'",
                    group_name, property_name));
  }

  // Error case: property value is not positive.
  if (bad_value.has_value()) {
    ProximityProperties negative_value(props);
    negative_value.AddProperty(group_name, property_name, *bad_value);
    DRAKE_EXPECT_THROWS_MESSAGE(
        maker(shape_spec, negative_value), std::logic_error,
        fmt::format("Cannot create {} {}.+'{}'.+ positive", compliance,
                    shape_name, property_name));
  }
}

// TODO(SeanCurtis-TRI): Add Cylinder, Mesh, Capsule, Ellipsoid and Convex as
//  they become supported by either rigid or soft geometries.

// Test suite for testing the common failure conditions for generating rigid
// geometry. Specifically, they just need to be tessellated into a triangle mesh
// and, therefore, only depend on the (hydroelastic, characteristic_length)
// value. This actively excludes Box, Convex, HalfSpace and Mesh because they
// don't depend on any of the proximity properties (see the
// `RigidErrorShapeTypes` declaration below.) It should include every *other*
// supported rigid shape type.
template <typename ShapeType>
class HydroelasticRigidGeometryErrorTests : public ::testing::Test {};

TYPED_TEST_SUITE_P(HydroelasticRigidGeometryErrorTests);

TYPED_TEST_P(HydroelasticRigidGeometryErrorTests, BadResolutionHint) {
  using ShapeType = TypeParam;
  ShapeType shape_spec = make_default_shape<ShapeType>();

  TestPropertyErrors<ShapeType, double>(
      shape_spec, kHydroGroup, kRezHint, "rigid",
      [](const ShapeType& s, const ProximityProperties& p) {
        MakeRigidRepresentation(s, p);
      },
      -0.2, {});
}

REGISTER_TYPED_TEST_SUITE_P(HydroelasticRigidGeometryErrorTests,
                            BadResolutionHint);
typedef ::testing::Types<Sphere, Capsule, Cylinder, Ellipsoid>
    RigidErrorShapeTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, HydroelasticRigidGeometryErrorTests,
                              RigidErrorShapeTypes);

class HydroelasticSoftGeometryTest : public ::testing::Test {
 protected:
  /* Creates a simple set of properties for generating soft geometry. */
  ProximityProperties soft_properties(double edge_length = 0.1) const {
    ProximityProperties soft_properties;
    AddSoftHydroelasticProperties(edge_length, 1e8, &soft_properties);
    return soft_properties;
  }
};

// TODO(SeanCurtis-TRI): As new shape specifications are added, they are
//  implicitly unsupported and should be added here (and in
//  UnsupportedRigidShapes).
// Smoke test for shapes that are *known* to be unsupported as soft objects.
// NOTE: This will spew warnings to the log.
TEST_F(HydroelasticSoftGeometryTest, UnsupportedSoftShapes) {
  ProximityProperties props = soft_properties();

  // Note: the file name doesn't have to be valid for this (and the Mesh) test.
  const std::string obj = "drake/geometry/proximity/test/no_such_files.obj";
  EXPECT_EQ(MakeSoftRepresentation(Convex(obj, 1.0), props), std::nullopt);

  EXPECT_EQ(MakeSoftRepresentation(Mesh(obj, 1.0), props), std::nullopt);
}

TEST_F(HydroelasticSoftGeometryTest, HalfSpace) {
  ProximityProperties properties = soft_properties();

  // Case: A half space without (hydroelastic, slab_thickness) throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSoftRepresentation(HalfSpace(), properties), std::logic_error,
      "Cannot create soft HalfSpace; missing the .*slab_thickness.* property");

  // Case: fully specified half space.
  const double thickness = 1.3;
  properties.AddProperty(kHydroGroup, kSlabThickness, thickness);
  std::optional<SoftGeometry> half_space =
      MakeSoftRepresentation(HalfSpace(), properties);
  ASSERT_NE(half_space, std::nullopt);
  EXPECT_TRUE(half_space->is_half_space());
  EXPECT_EQ(
      half_space->pressure_scale(),
      properties.GetProperty<double>(kHydroGroup, kElastic) / thickness);

  DRAKE_EXPECT_THROWS_MESSAGE(
      half_space->mesh(), std::runtime_error,
      "SoftGeometry::mesh.* cannot be invoked .* half space");
  DRAKE_EXPECT_THROWS_MESSAGE(
      half_space->pressure_field(), std::runtime_error,
      "SoftGeometry::pressure.* cannot be invoked .* half space");
  DRAKE_EXPECT_THROWS_MESSAGE(
      half_space->bvh(), std::runtime_error,
      "SoftGeometry::bvh.* cannot be invoked .* half space");
}

// Test construction of a soft sphere. Confirms that the edge length has
// an effect (i.e., that shrinking the edge_length by at least half causes
// a change in mesh resolution. It relies on unit tests for the unit sphere
// generation to confirm that it's the *right* number of tetrahedron. This
// merely confirms that the characteristic_length is being fed in.
TEST_F(HydroelasticSoftGeometryTest, Sphere) {
  const double kRadius = 0.5;
  Sphere sphere_spec(kRadius);

  // Confirm that characteristic length is being fed in properly -- i.e.,
  // if characteristic length cuts in half, It should have more tetrahedra.
  ProximityProperties properties1 = soft_properties(kRadius);
  ProximityProperties properties2 = soft_properties(kRadius / 2);
  std::optional<SoftGeometry> sphere1 =
      MakeSoftRepresentation(sphere_spec, properties1);
  std::optional<SoftGeometry> sphere2 =
      MakeSoftRepresentation(sphere_spec, properties2);
  EXPECT_FALSE(sphere1->is_half_space());
  EXPECT_FALSE(sphere2->is_half_space());
  EXPECT_LT(sphere1->mesh().num_elements(), sphere2->mesh().num_elements());
  // This is the only test where we confirm that bvh() *doesn't* throw for
  // meshes and slab_thickness() does.
  EXPECT_NO_THROW(sphere1->bvh());
  DRAKE_EXPECT_THROWS_MESSAGE(
      sphere1->pressure_scale(), std::runtime_error,
      "SoftGeometry::pressure_scale.* cannot be invoked .* soft mesh");

  // Confirm that all vertices lie inside the sphere and that at least one lies
  // on the boundary.
  double max_distance = -1.0;
  for (const auto& soft_geometry : {*sphere1, *sphere2}) {
    const VolumeMesh<double>& mesh = soft_geometry.mesh();
    for (int v = 0; v < mesh.num_vertices(); ++v) {
      const double dist = mesh.vertex(v).norm();
      max_distance = std::max(max_distance, dist);
      ASSERT_LE(dist, kRadius);
    }
  }

  ASSERT_NEAR(max_distance, kRadius, 1e-15);

  // Confirm pressure field is as specified in the properties.
  const double E =
      properties1.GetPropertyOrDefault(kHydroGroup, kElastic, 1e8);
  // We assume that the sphere's pressure is defined as E * (1 - r/R).
  auto pressure = [E, kRadius](const Vector3d& r_MV) {
    return E * (1.0 - r_MV.norm() / kRadius);
  };
  const double kEps = std::numeric_limits<double>::epsilon();
  const VolumeMesh<double>& mesh = sphere1->mesh();
  for (int v = 0; v < mesh.num_vertices(); ++v) {
    const Vector3d& vertex = mesh.vertex(v);
    // Zero on outside, 1 on inside.
    const double expected_p = pressure(vertex);
    EXPECT_NEAR(sphere1->pressure_field().EvaluateAtVertex(v), expected_p,
                kEps * E);
  }

  // Confirm that it respects the ("hydroelastic", "tessellation_strategy")
  // property in the following ways:
  {
      // It defaults to single-interior-vertex if nothing is defined.

      // Sphere 1 and sphere 2 have resolution hints that differ by a factor
      // of two --> sphere 2's level of refinement is one greater than sphere
      // 1's. Both are missing the "tessellation_strategy" property so it should
      // default to kSingleInteriorVertex. So, sphere 2 must have 4X the
      // tetrahedra as sphere 1.
      EXPECT_EQ(sphere1->mesh().num_elements() * 4,
                sphere2->mesh().num_elements());
  }

  {
    // Defining kDenseInteriorVertices produces a mesh with an increased number
    // of tets (compared to an otherwise identical mesh declared to sparse).

    // Starting with sphere 1's properties, we'll set it to dense and observe
    // more tets.
    ProximityProperties dense_properties(properties1);
    dense_properties.AddProperty(kHydroGroup, "tessellation_strategy",
                                 TessellationStrategy::kDenseInteriorVertices);
    std::optional<SoftGeometry> dense_sphere =
        MakeSoftRepresentation(sphere_spec, dense_properties);
    EXPECT_LT(sphere1->mesh().num_elements(),
              dense_sphere->mesh().num_elements());
  }

  {
    // Explicitly defining kSingleInteriorVertex still produces sparse.

    // Starting with sphere 1's properties, we'll explicitly set it to sparse
    // and observe the same number of tets.
    ProximityProperties dense_properties(properties1);
    dense_properties.AddProperty(kHydroGroup, "tessellation_strategy",
                                 TessellationStrategy::kSingleInteriorVertex);
    std::optional<SoftGeometry> dense_sphere =
        MakeSoftRepresentation(sphere_spec, dense_properties);
    EXPECT_EQ(sphere1->mesh().num_elements(),
              dense_sphere->mesh().num_elements());
  }

  {
    // A value that isn't a TessellationStrategy throws.
    // Starting with sphere 1's properties, we'll set the property to be a
    // string. Should throw.
    ProximityProperties dense_properties(properties1);
    dense_properties.AddProperty(kHydroGroup, "tessellation_strategy", "dense");
    EXPECT_THROW(MakeSoftRepresentation(sphere_spec, dense_properties),
                 std::logic_error);
  }
}

// Test construction of a soft box.
TEST_F(HydroelasticSoftGeometryTest, Box) {
  const Box box_spec(0.2, 0.4, 0.8);

  ProximityProperties properties = soft_properties();
  std::optional<SoftGeometry> box =
      MakeSoftRepresentation(box_spec, properties);

  // Smoke test the mesh and the pressure field. It relies on unit tests for
  // the generators of the mesh and the pressure field.
  const int expected_num_vertices = 12;
  EXPECT_EQ(box->mesh().num_vertices(), expected_num_vertices);
  const double E =
      properties.GetPropertyOrDefault(kHydroGroup, kElastic, 1e8);
  for (int v = 0; v < box->mesh().num_vertices(); ++v) {
    const double pressure = box->pressure_field().EvaluateAtVertex(v);
    EXPECT_GE(pressure, 0);
    EXPECT_LE(pressure, E);
  }
}

// Test construction of a soft cylinder.
TEST_F(HydroelasticSoftGeometryTest, Cylinder) {
  const double radius = 1.0;
  const double length = 2.0;
  const Cylinder cylinder_spec(radius, length);

  // Confirm that characteristic length is being fed in properly. Pick a
  // characteristic length *larger* than the cylinder dimensions to get the
  // coarsest mesh with 15 vertices.
  ProximityProperties properties = soft_properties(1.5 * length);
  std::optional<SoftGeometry> cylinder =
      MakeSoftRepresentation(cylinder_spec, properties);

  // Smoke test the mesh and the pressure field. It relies on unit tests for
  // the generators of the mesh and the pressure field.
  const int expected_num_vertices = 9;
  EXPECT_EQ(cylinder->mesh().num_vertices(), expected_num_vertices);
  const double E =
      properties.GetPropertyOrDefault(kHydroGroup, kElastic, 1e8);
  for (int v = 0; v < cylinder->mesh().num_vertices(); ++v) {
    const double pressure = cylinder->pressure_field().EvaluateAtVertex(v);
    EXPECT_GE(pressure, 0);
    EXPECT_LE(pressure, E);
  }
}

// Test construction of a soft capsule.
TEST_F(HydroelasticSoftGeometryTest, Capsule) {
  const double radius = 1.0;
  const double length = 2.0;
  const Capsule capsule_spec(radius, length);

  // Confirm that characteristic length is being fed in properly. Pick a
  // characteristic length *larger* than the capsule dimensions to get the
  // coarsest mesh with 10 vertices.
  ProximityProperties properties = soft_properties(1.5 * length);
  std::optional<SoftGeometry> capsule =
      MakeSoftRepresentation(capsule_spec, properties);

  // Smoke test the mesh and the pressure field. It relies on unit tests for
  // the generators of the mesh and the pressure field.
  const int expected_num_vertices = 10;
  EXPECT_EQ(capsule->mesh().num_vertices(), expected_num_vertices);
  // TODO(joemasterjohn): Change all instances of `GetPropertyOrDefault` to
  // `GetProperty` variant.
  const double E =
      properties.GetPropertyOrDefault(kHydroGroup, kElastic, 1e8);
  for (int v = 0; v < capsule->mesh().num_vertices(); ++v) {
    const double pressure = capsule->pressure_field().EvaluateAtVertex(v);
    EXPECT_GE(pressure, 0);
    EXPECT_LE(pressure, E);
  }
}

// Test construction of a soft ellipsoid.
TEST_F(HydroelasticSoftGeometryTest, Ellipsoid) {
  // Lengths of the three semi-principal axes of the ellipsoid:
  //     (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
  const double a = 0.05;
  const double b = 0.08;
  const double c = 0.03;
  const Ellipsoid ellipsoid_spec(a, b, c);

  // Confirm that characteristic length is being fed in properly. Pick a
  // characteristic length *larger* than the ellipsoid dimensions to get the
  // coarsest mesh with 7 vertices.
  ProximityProperties properties = soft_properties(0.16);
  std::optional<SoftGeometry> ellipsoid =
      MakeSoftRepresentation(ellipsoid_spec, properties);

  // Smoke test the mesh and the pressure field. It relies on unit tests for
  // the generators of the mesh and the pressure field.
  const int expected_num_vertices = 7;
  EXPECT_EQ(ellipsoid->mesh().num_vertices(), expected_num_vertices);
  const double E =
      properties.GetPropertyOrDefault(kHydroGroup, kElastic, 1e8);
  for (int v = 0; v < ellipsoid->mesh().num_vertices(); ++v) {
    const double pressure = ellipsoid->pressure_field().EvaluateAtVertex(v);
    EXPECT_GE(pressure, 0);
    EXPECT_LE(pressure, E);
  }

  // The remaining tests confirm that it respects the
  // ("hydroelastic", "tessellation_strategy") property.

  ProximityProperties basic_properties = soft_properties(0.08);
  ProximityProperties sparse_properties(basic_properties);
  sparse_properties.AddProperty(kHydroGroup, "tessellation_strategy",
                                TessellationStrategy::kSingleInteriorVertex);
  ProximityProperties dense_properties(basic_properties);
  dense_properties.AddProperty(kHydroGroup, "tessellation_strategy",
                               TessellationStrategy::kDenseInteriorVertices);

  std::optional<SoftGeometry> implicit_sparse_ellipsoid =
      MakeSoftRepresentation(ellipsoid_spec, basic_properties);
  std::optional<SoftGeometry> sparse_ellipsoid =
      MakeSoftRepresentation(ellipsoid_spec, sparse_properties);
  std::optional<SoftGeometry> dense_ellipsoid =
      MakeSoftRepresentation(ellipsoid_spec, dense_properties);

  {
    // It defaults to kSingleInteriorVertex if nothing is defined.

    // The implicitly sparse ellipsoid should have the same number of tets
    // as that declared explicitly.
    EXPECT_EQ(implicit_sparse_ellipsoid->mesh().num_elements(),
              sparse_ellipsoid->mesh().num_elements());
  }

  {
    // Explicitly specifying the two strategies produces meshes with different
    // numbers of tets.

    // The dense ellipsoid (with the same resolution hint) should have more
    // tets.
    EXPECT_LT(sparse_ellipsoid->mesh().num_elements(),
              dense_ellipsoid->mesh().num_elements());
  }

  {
    // A value that isn't a TessellationStrategy throws.

    // Starting with the basic properties, we'll set the property to be a
    // string. Should throw.
    ProximityProperties bad_properties(basic_properties);
    bad_properties.AddProperty(kHydroGroup, "tessellation_strategy", "dense");
    EXPECT_THROW(MakeSoftRepresentation(ellipsoid_spec, bad_properties),
                 std::logic_error);
  }
}

// Test suite for testing the common failure conditions for generating soft
// geometry. Specifically, they need to be tessellated into a tet mesh
// and define a pressure field. This actively excludes Convex and Mesh
// because soft Convex and soft Mesh are not currently supported for
// hydroelastic contact. (See the `SoftErrorShapeTypes` declaration below.)
// It should include every *other* supported soft shape type. For HalfSpace
// and Box, they are included in the test suite but exempt from
// BadResolutionHint because they do not depend on the resolution hint
// parameter. Only HalfSpace is tested in BadSlabThickness.
template <typename ShapeType>
class HydroelasticSoftGeometryErrorTests : public ::testing::Test {};

TYPED_TEST_SUITE_P(HydroelasticSoftGeometryErrorTests);

TYPED_TEST_P(HydroelasticSoftGeometryErrorTests, BadResolutionHint) {
  using ShapeType = TypeParam;
  ShapeType shape_spec = make_default_shape<ShapeType>();
  if (ShapeName(shape_spec).name() != "HalfSpace" &&
      ShapeName(shape_spec).name() != "Box") {
    TestPropertyErrors<ShapeType, double>(
        shape_spec, kHydroGroup, kRezHint, "soft",
        [](const ShapeType& s, const ProximityProperties& p) {
          MakeSoftRepresentation(s, p);
        },
        -0.2, {});
  }
}

TYPED_TEST_P(HydroelasticSoftGeometryErrorTests, BadElasticModulus) {
  using ShapeType = TypeParam;
  ShapeType shape_spec = make_default_shape<ShapeType>();

  ProximityProperties soft_properties;
  // Add the resolution hint and slab thickness, so that creation of the
  // hydroelastic representation can choke on elastic modulus value.
  soft_properties.AddProperty(kHydroGroup, kRezHint, 10.0);
  soft_properties.AddProperty(kHydroGroup, kSlabThickness, 1.0);
  TestPropertyErrors<ShapeType, double>(
      shape_spec, kHydroGroup, kElastic, "soft",
      [](const ShapeType& s, const ProximityProperties& p) {
        MakeSoftRepresentation(s, p);
      },
      -0.2, soft_properties);
}

TYPED_TEST_P(HydroelasticSoftGeometryErrorTests, BadSlabThickness) {
  using ShapeType = TypeParam;
  ShapeType shape_spec = make_default_shape<ShapeType>();
  // Half space only!
  if (ShapeName(shape_spec).name() == "HalfSpace") {
    TestPropertyErrors<ShapeType, double>(
        shape_spec, kHydroGroup, kSlabThickness, "soft",
        [](const ShapeType& s, const ProximityProperties& p) {
          MakeSoftRepresentation(s, p);
        },
        -0.2, {});
  }
}

REGISTER_TYPED_TEST_SUITE_P(HydroelasticSoftGeometryErrorTests,
                            BadResolutionHint, BadElasticModulus,
                            BadSlabThickness);
typedef ::testing::Types<Sphere, Box, Capsule, Cylinder, Ellipsoid, HalfSpace>
    SoftErrorShapeTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, HydroelasticSoftGeometryErrorTests,
                               SoftErrorShapeTypes);

}  // namespace
}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
