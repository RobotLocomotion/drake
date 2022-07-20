#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/deformable_mesh_intersection.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

class GeometriesTester {
 public:
  /* Returns the rigid geometry with `rigid_id` registered in `geometries`.
   @pre a rigid representation with `rigid_id` exists in `geometries`. */
  static const RigidGeometry& get_rigid_geometry(const Geometries& geometries,
                                                 GeometryId rigid_id) {
    return geometries.rigid_geometries_.at(rigid_id);
  }
};

namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

/* Makes a ProximityProperties with a resolution hint property in the hydro
 group.
 @pre resolution_hint > 0. */
ProximityProperties MakeProximityPropsWithRezHint(double resolution_hint) {
  ProximityProperties props;
  props.AddProperty(internal::kHydroGroup, internal::kRezHint, resolution_hint);
  return props;
}

GTEST_TEST(GeometriesTest, AddRigidGeometry) {
  Geometries geometries;
  GeometryId rigid_id = GeometryId::get_new_id();
  /* No geometries have been added yet. */
  EXPECT_FALSE(geometries.is_rigid(rigid_id));

  /* Add a rigid geometry with resolution hint property. */
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeProximityPropsWithRezHint(kRezHint);
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id, props);

  EXPECT_TRUE(geometries.is_rigid(rigid_id));

  /* Trying to a rigid geometry without the resolution hint property is a no-op.
   */
  GeometryId g_id = GeometryId::get_new_id();
  ProximityProperties empty_props;
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), g_id, empty_props);

  EXPECT_FALSE(geometries.is_rigid(g_id));
}

/* Test coverage for all unsupported shapes as rigid geometries: MeshcatCone and
 * HalfSpace. */
GTEST_TEST(GeometriesTest, UnsupportedRigidShapes) {
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeProximityPropsWithRezHint(kRezHint);
  Geometries geometries;

  /* Unsupported shapes: MeshcatCone, HalfSpace. */
  /* MeshcatCone */
  {
    GeometryId cone_id = GeometryId::get_new_id();
    const double height = 2.0;
    const double a = 1.0;
    const double b = 1.0;
    EXPECT_NO_THROW(geometries.MaybeAddRigidGeometry(MeshcatCone(height, a, b),
                                                     cone_id, props));
    EXPECT_FALSE(geometries.is_rigid(cone_id));
  }
  /* HalfSpace */
  {
    GeometryId hs_id = GeometryId::get_new_id();
    EXPECT_NO_THROW(
        geometries.MaybeAddRigidGeometry(HalfSpace(), hs_id, props));
    EXPECT_FALSE(geometries.is_rigid(hs_id));
  }
}

/* Test coverage for all supported shapes as rigid geometries: Box, Sphere,
 Cylinder, Capsule, Ellipsoid, Mesh, Convex. */
GTEST_TEST(GeometriesTest, SupportedRigidShapes) {
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeProximityPropsWithRezHint(kRezHint);
  Geometries geometries;

  /* Box */
  {
    GeometryId box_id = GeometryId::get_new_id();
    geometries.MaybeAddRigidGeometry(Box::MakeCube(1.0), box_id, props);
    EXPECT_TRUE(geometries.is_rigid(box_id));
  }
  /* Sphere */
  {
    const double radius = 1.0;
    GeometryId box_id = GeometryId::get_new_id();
    geometries.MaybeAddRigidGeometry(Sphere(radius), box_id, props);
    EXPECT_TRUE(geometries.is_rigid(box_id));
  }
  /* Cylinder */
  {
    GeometryId cylinder_id = GeometryId::get_new_id();
    const double radius = 1.0;
    const double length = 2.0;
    geometries.MaybeAddRigidGeometry(Cylinder(radius, length), cylinder_id,
                                     props);
    EXPECT_TRUE(geometries.is_rigid(cylinder_id));
  }
  /* Capsule */
  {
    GeometryId capsule_id = GeometryId::get_new_id();
    const double radius = 1.0;
    const double length = 2.0;
    geometries.MaybeAddRigidGeometry(Capsule(radius, length), capsule_id,
                                     props);
    EXPECT_TRUE(geometries.is_rigid(capsule_id));
  }
  /* Ellipsoid */
  {
    GeometryId ellipsoid_id = GeometryId::get_new_id();
    const double a = 0.5;
    const double b = 0.8;
    const double c = 0.3;
    geometries.MaybeAddRigidGeometry(Ellipsoid(a, b, c), ellipsoid_id, props);
    EXPECT_TRUE(geometries.is_rigid(ellipsoid_id));
  }
  /* Mesh */
  {
    GeometryId mesh_id = GeometryId::get_new_id();
    std::string file = FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    geometries.MaybeAddRigidGeometry(Mesh(file, 1.0), mesh_id, props);
    EXPECT_TRUE(geometries.is_rigid(mesh_id));
  }
  /* Convex */
  {
    GeometryId convex_id = GeometryId::get_new_id();
    std::string file = FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    geometries.MaybeAddRigidGeometry(Convex(file, 1.0), convex_id, props);
    EXPECT_TRUE(geometries.is_rigid(convex_id));
  }
}

GTEST_TEST(GeometriesTest, UpdateRigidWorldPose) {
  Geometries geometries;

  /* Add a rigid geometry. */
  GeometryId rigid_id = GeometryId::get_new_id();
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeProximityPropsWithRezHint(kRezHint);
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id, props);

  /* Initially the pose is identity. */
  {
    const RigidGeometry& rigid_geometry =
        GeometriesTester::get_rigid_geometry(geometries, rigid_id);
    EXPECT_TRUE(rigid_geometry.pose_in_world().IsExactlyIdentity());
  }
  /* Update the pose to some arbitrary value. */
  const math::RigidTransform<double> X_WG(
      math::RollPitchYaw<double>(-1.57, 0, 3), Vector3d(-0.3, -0.55, 0.36));
  geometries.UpdateRigidWorldPose(rigid_id, X_WG);
  {
    const RigidGeometry& rigid_geometry =
        GeometriesTester::get_rigid_geometry(geometries, rigid_id);
    EXPECT_TRUE(rigid_geometry.pose_in_world().IsExactlyEqualTo(X_WG));
  }
}

GTEST_TEST(GeometriesTest, RemoveGeometry) {
  Geometries geometries;

  /* Add a couple of rigid geometries. */
  GeometryId rigid_id0 = GeometryId::get_new_id();
  GeometryId rigid_id1 = GeometryId::get_new_id();
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeProximityPropsWithRezHint(kRezHint);
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id0, props);
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id1, props);

  /* Calling RemoveGeometry on an existing rigid geometry. */
  {
    geometries.RemoveGeometry(rigid_id0);
    /* The geometry is indeed removed. */
    EXPECT_FALSE(geometries.is_rigid(rigid_id0));
    /* Other geometries are unaffected. */
    EXPECT_TRUE(geometries.is_rigid(rigid_id1));
  }
  /* Calling RemoveGeometry on an invalid or already deleted geometry is a
   no-op. */
  {
    GeometryId invalid_id = GeometryId::get_new_id();
    EXPECT_NO_THROW(geometries.RemoveGeometry(invalid_id));
    EXPECT_NO_THROW(geometries.RemoveGeometry(rigid_id0));

    EXPECT_FALSE(geometries.is_rigid(rigid_id0));
    EXPECT_TRUE(geometries.is_rigid(rigid_id1));
  }
}

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
