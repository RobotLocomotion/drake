#include "drake/multibody/fixed_fem/dev/collision_objects.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_capsule_mesh.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {
using geometry::Box;
using geometry::Capsule;
using geometry::Convex;
using geometry::Cylinder;
using geometry::Ellipsoid;
using geometry::GeometryId;
using geometry::HalfSpace;
using geometry::Mesh;
using geometry::ProximityProperties;
using geometry::ReadObjToSurfaceMesh;
using geometry::Shape;
using geometry::Sphere;
using geometry::SurfaceMesh;
using geometry::internal::MakeBoxSurfaceMesh;
using geometry::internal::MakeCapsuleSurfaceMesh;
using geometry::internal::MakeCylinderSurfaceMesh;
using geometry::internal::MakeEllipsoidSurfaceMesh;
using geometry::internal::MakeSphereSurfaceMesh;

const char* const dummy_group_name = "group";
const char* const dummy_property_name = "property";
const double dummy_property_value = 3.14;

class CollisionObjectsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Make some dummy proximity properties.
    proximity_properties_.AddProperty(dummy_group_name, dummy_property_name,
                                      dummy_property_value);
  }

  void VerifyProximityProperties(GeometryId id) const {
    const double retrieved_property =
        collision_objects_.proximity_properties(id).GetProperty<double>(
            dummy_group_name, dummy_property_name);
    const double expected_property = proximity_properties_.GetProperty<double>(
        dummy_group_name, dummy_property_name);
    EXPECT_EQ(retrieved_property, expected_property);
  }

  double resolution_hint() const { return collision_objects_.resolution_hint_; }

  CollisionObjects<double> collision_objects_;
  ProximityProperties proximity_properties_;
};

namespace {
TEST_F(CollisionObjectsTest, AddSphere) {
  GeometryId id = GeometryId::get_new_id();
  Sphere sphere(0.123);
  collision_objects_.AddCollisionObject(id, sphere, proximity_properties_);

  VerifyProximityProperties(id);

  const SurfaceMesh<double> expected_surface_mesh =
      MakeSphereSurfaceMesh<double>(sphere, resolution_hint());
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects_.mesh(id)));
}

TEST_F(CollisionObjectsTest, AddBox) {
  GeometryId id = GeometryId::get_new_id();
  Box box(0.123, 0.456, 0.789);
  collision_objects_.AddCollisionObject(id, box, proximity_properties_);

  VerifyProximityProperties(id);

  // The box mesh is always as coarse as possible.
  const SurfaceMesh<double> expected_surface_mesh =
      MakeBoxSurfaceMesh<double>(box, 1e10);
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects_.mesh(id)));
}

TEST_F(CollisionObjectsTest, AddCylinder) {
  GeometryId id = GeometryId::get_new_id();
  Cylinder cylinder(0.123, 0.456);
  collision_objects_.AddCollisionObject(id, cylinder, proximity_properties_);

  VerifyProximityProperties(id);

  const SurfaceMesh<double> expected_surface_mesh =
      MakeCylinderSurfaceMesh<double>(cylinder, resolution_hint());
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects_.mesh(id)));
}

TEST_F(CollisionObjectsTest, AddCapsule) {
  GeometryId id = GeometryId::get_new_id();
  Capsule capsule(0.123, 0.456);
  collision_objects_.AddCollisionObject(id, capsule, proximity_properties_);

  VerifyProximityProperties(id);

  const SurfaceMesh<double> expected_surface_mesh =
      MakeCapsuleSurfaceMesh<double>(capsule, resolution_hint());
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects_.mesh(id)));
}

TEST_F(CollisionObjectsTest, AddEllipsoid) {
  GeometryId id = GeometryId::get_new_id();
  Ellipsoid ellipsoid(0.123, 0.456, 0.789);
  collision_objects_.AddCollisionObject(id, ellipsoid, proximity_properties_);

  VerifyProximityProperties(id);

  const SurfaceMesh<double> expected_surface_mesh =
      MakeEllipsoidSurfaceMesh<double>(ellipsoid, resolution_hint());
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects_.mesh(id)));
}

TEST_F(CollisionObjectsTest, AddConvex) {
  GeometryId id = GeometryId::get_new_id();
  Convex convex{drake::FindResourceOrThrow(
      "drake/multibody/fixed_fem/dev/test/quad_cube.obj")};
  collision_objects_.AddCollisionObject(id, convex, proximity_properties_);

  VerifyProximityProperties(id);

  const SurfaceMesh<double> expected_surface_mesh =
      ReadObjToSurfaceMesh(convex.filename(), convex.scale());
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects_.mesh(id)));
}

TEST_F(CollisionObjectsTest, AddMesh) {
  GeometryId id = GeometryId::get_new_id();
  Mesh mesh{drake::FindResourceOrThrow(
      "drake/multibody/fixed_fem/dev/test/non_convex_mesh.obj")};
  collision_objects_.AddCollisionObject(id, mesh, proximity_properties_);

  VerifyProximityProperties(id);

  const SurfaceMesh<double> expected_surface_mesh =
      ReadObjToSurfaceMesh(mesh.filename(), mesh.scale());
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects_.mesh(id)));
}

/* Exercises pose getter and setter. */
TEST_F(CollisionObjectsTest, Poses) {
  GeometryId id = GeometryId::get_new_id();
  Box box(0.123, 0.456, 0.789);
  collision_objects_.AddCollisionObject(id, box, proximity_properties_);
  const auto identity = math::RigidTransform<double>();
  EXPECT_TRUE(collision_objects_.pose(id).IsExactlyEqualTo(identity));
  math::RigidTransform<double> arbitrary_pose(Vector3<double>(1., 2., 3.));
  collision_objects_.UpdatePoseInWorld(id, arbitrary_pose);
  EXPECT_TRUE(collision_objects_.pose(id).IsExactlyEqualTo(arbitrary_pose));
}
}  // namespace
}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
