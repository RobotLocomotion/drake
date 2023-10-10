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

/* Use GeometrySetTester's friend status with GeometrySet to leak its geometry
 ids to support the tests below. */
class GeometrySetTester {
 public:
  static std::unordered_set<GeometryId> geometries(const GeometrySet& s,
                                                   CollisionFilterScope) {
    return s.geometries();
  }
};

namespace internal {
namespace deformable {

class GeometriesTester {
 public:
  /* Returns the deformable geometry with `deformable_id` registered in
   `geometries`.
   @pre a deformable representation with `deformable_id` exists in `geometries`.
  */
  static const DeformableGeometry& get_deformable_geometry(
      const Geometries& geometries, GeometryId deformable_id) {
    return geometries.deformable_geometries_.at(deformable_id);
  }

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

/* Makes an arbitrary volume mesh. */
VolumeMesh<double> MakeVolumeMesh() {
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  Sphere sphere(kRadius);
  return MakeSphereVolumeMesh<double>(
      sphere, kRezHint, TessellationStrategy::kDenseInteriorVertices);
}

/* Makes a ProximityProperties with a resolution hint property in the hydro
 group.
 @pre resolution_hint > 0. */
ProximityProperties MakeProximityPropsWithRezHint(double resolution_hint) {
  ProximityProperties props;
  props.AddProperty(internal::kHydroGroup, internal::kRezHint, resolution_hint);
  return props;
}

math::RigidTransformd default_pose() {
  return math::RigidTransformd(math::RollPitchYaw<double>(1, 2, 3),
                               Vector3d(4, 5, 6));
}

/* Returns a value for the collision filter id extraction functor. */
static CollisionFilter::ExtractIds get_extract_ids_functor() {
  return &GeometrySetTester::geometries;
}

GTEST_TEST(GeometriesTest, AddRigidGeometry) {
  Geometries geometries;
  GeometryId rigid_id = GeometryId::get_new_id();
  /* No geometries have been added yet. */
  EXPECT_FALSE(geometries.is_rigid(rigid_id));
  EXPECT_FALSE(geometries.is_deformable(rigid_id));

  /* Add a rigid geometry with resolution hint property. */
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeProximityPropsWithRezHint(kRezHint);
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id, props,
                                   default_pose());

  EXPECT_TRUE(geometries.is_rigid(rigid_id));
  EXPECT_FALSE(geometries.is_deformable(rigid_id));

  /* Trying to a rigid geometry without the resolution hint property is a no-op.
   */
  GeometryId g_id = GeometryId::get_new_id();
  ProximityProperties empty_props;
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), g_id, empty_props,
                                   default_pose());

  EXPECT_FALSE(geometries.is_rigid(g_id));
  EXPECT_FALSE(geometries.is_deformable(g_id));
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
    EXPECT_NO_THROW(geometries.MaybeAddRigidGeometry(
        MeshcatCone(height, a, b), cone_id, props, default_pose()));
    EXPECT_FALSE(geometries.is_rigid(cone_id));
  }
  /* HalfSpace */
  {
    GeometryId hs_id = GeometryId::get_new_id();
    EXPECT_NO_THROW(geometries.MaybeAddRigidGeometry(HalfSpace(), hs_id, props,
                                                     default_pose()));
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
    geometries.MaybeAddRigidGeometry(Box::MakeCube(1.0), box_id, props,
                                     default_pose());
    EXPECT_TRUE(geometries.is_rigid(box_id));
  }
  /* Sphere */
  {
    const double radius = 1.0;
    GeometryId box_id = GeometryId::get_new_id();
    geometries.MaybeAddRigidGeometry(Sphere(radius), box_id, props,
                                     default_pose());
    EXPECT_TRUE(geometries.is_rigid(box_id));
  }
  /* Cylinder */
  {
    GeometryId cylinder_id = GeometryId::get_new_id();
    const double radius = 1.0;
    const double length = 2.0;
    geometries.MaybeAddRigidGeometry(Cylinder(radius, length), cylinder_id,
                                     props, default_pose());
    EXPECT_TRUE(geometries.is_rigid(cylinder_id));
  }
  /* Capsule */
  {
    GeometryId capsule_id = GeometryId::get_new_id();
    const double radius = 1.0;
    const double length = 2.0;
    geometries.MaybeAddRigidGeometry(Capsule(radius, length), capsule_id, props,
                                     default_pose());
    EXPECT_TRUE(geometries.is_rigid(capsule_id));
  }
  /* Ellipsoid */
  {
    GeometryId ellipsoid_id = GeometryId::get_new_id();
    const double a = 0.5;
    const double b = 0.8;
    const double c = 0.3;
    geometries.MaybeAddRigidGeometry(Ellipsoid(a, b, c), ellipsoid_id, props,
                                     default_pose());
    EXPECT_TRUE(geometries.is_rigid(ellipsoid_id));
  }
  /* Mesh */
  {
    GeometryId mesh_id = GeometryId::get_new_id();
    std::string file = FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    geometries.MaybeAddRigidGeometry(Mesh(file, 1.0), mesh_id, props,
                                     default_pose());
    EXPECT_TRUE(geometries.is_rigid(mesh_id));
  }
  /* Convex */
  {
    GeometryId convex_id = GeometryId::get_new_id();
    std::string file = FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    geometries.MaybeAddRigidGeometry(Convex(file, 1.0), convex_id, props,
                                     default_pose());
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
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id, props,
                                   default_pose());

  /* Initially the pose is the default pose. */
  {
    const RigidGeometry& rigid_geometry =
        GeometriesTester::get_rigid_geometry(geometries, rigid_id);
    EXPECT_TRUE(
        rigid_geometry.pose_in_world().IsExactlyEqualTo(default_pose()));
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

GTEST_TEST(GeometriesTest, AddDeformableGeometry) {
  Geometries geometries;
  GeometryId deformable_id = GeometryId::get_new_id();
  EXPECT_FALSE(geometries.is_rigid(deformable_id));
  EXPECT_FALSE(geometries.is_deformable(deformable_id));

  /* Add a deformable geometry. */
  geometries.AddDeformableGeometry(deformable_id, MakeVolumeMesh());
  EXPECT_FALSE(geometries.is_rigid(deformable_id));
  EXPECT_TRUE(geometries.is_deformable(deformable_id));
}

GTEST_TEST(GeometriesTest, RemoveGeometry) {
  Geometries geometries;
  /* Add a couple of deformable geometries. */
  GeometryId deformable_id0 = GeometryId::get_new_id();
  GeometryId deformable_id1 = GeometryId::get_new_id();
  geometries.AddDeformableGeometry(deformable_id0, MakeVolumeMesh());
  geometries.AddDeformableGeometry(deformable_id1, MakeVolumeMesh());

  /* Add a couple of rigid geometries. */
  GeometryId rigid_id0 = GeometryId::get_new_id();
  GeometryId rigid_id1 = GeometryId::get_new_id();
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeProximityPropsWithRezHint(kRezHint);
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id0, props,
                                   default_pose());
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id1, props,
                                   default_pose());

  /* Calling RemoveGeometry on an existing deformable geometry. */
  geometries.RemoveGeometry(deformable_id0);
  /* The geometry is indeed removed. */
  EXPECT_FALSE(geometries.is_deformable(deformable_id0));
  /* Other geometries are unaffected. */
  EXPECT_TRUE(geometries.is_deformable(deformable_id1));
  EXPECT_TRUE(geometries.is_rigid(rigid_id0));
  EXPECT_TRUE(geometries.is_rigid(rigid_id1));

  /* Calling RemoveGeometry on an existing rigid geometry. */
  geometries.RemoveGeometry(rigid_id0);
  /* The geometry is indeed removed. */
  EXPECT_FALSE(geometries.is_rigid(rigid_id0));
  /* Other geometries are unaffected. */
  EXPECT_FALSE(geometries.is_deformable(deformable_id0));
  EXPECT_TRUE(geometries.is_deformable(deformable_id1));
  EXPECT_TRUE(geometries.is_rigid(rigid_id1));

  /* Calling RemoveGeometry on an invalid or already deleted geometry is a
   no-op. */
  GeometryId invalid_id = GeometryId::get_new_id();
  EXPECT_NO_THROW(geometries.RemoveGeometry(invalid_id));
  EXPECT_NO_THROW(geometries.RemoveGeometry(rigid_id0));

  EXPECT_FALSE(geometries.is_rigid(rigid_id0));
  EXPECT_FALSE(geometries.is_deformable(deformable_id0));
  EXPECT_TRUE(geometries.is_rigid(rigid_id1));
  EXPECT_TRUE(geometries.is_deformable(deformable_id1));
}

GTEST_TEST(GeometriesTest, UpdateDeformableVertexPositions) {
  Geometries geometries;
  /* Add a deformable geometry. */
  GeometryId deformable_id = GeometryId::get_new_id();
  const VolumeMesh<double> input_mesh = MakeVolumeMesh();
  geometries.AddDeformableGeometry(deformable_id, input_mesh);
  const int num_vertices = input_mesh.num_vertices();

  /* Initially the vertex positions is the same as the registered mesh. */
  {
    ASSERT_TRUE(geometries.is_deformable(deformable_id));
    const DeformableGeometry& geometry =
        GeometriesTester::get_deformable_geometry(geometries, deformable_id);
    EXPECT_TRUE(geometry.deformable_mesh().mesh().Equal(input_mesh));
  }
  /* Update the vertex positions to some arbitrary value. */
  const VectorXd q = VectorXd::LinSpaced(3 * num_vertices, 0.0, 1.0);
  geometries.UpdateDeformableVertexPositions(deformable_id, q);
  {
    const DeformableGeometry& geometry =
        GeometriesTester::get_deformable_geometry(geometries, deformable_id);
    const VolumeMesh<double>& mesh = geometry.deformable_mesh().mesh();
    for (int i = 0; i < num_vertices; ++i) {
      const Vector3d& q_MV = mesh.vertex(i);
      const Vector3d& reference_q_MV = input_mesh.vertex(i);
      const Vector3d& expected_q_MV = q.segment<3>(3 * i);
      EXPECT_EQ(q_MV, expected_q_MV);
      EXPECT_NE(q_MV, reference_q_MV);
    }
  }
}

GTEST_TEST(GeometriesTest, ComputeDeformableContact) {
  Geometries geometries;
  CollisionFilter collision_filter;
  /* The contact data is empty when there is no deformable geometry. */
  DeformableContact<double> contact_data =
      geometries.ComputeDeformableContact(collision_filter);
  EXPECT_EQ(contact_data.contact_surfaces().size(), 0);

  /* Add a deformable unit cube. */
  GeometryId deformable_id = GeometryId::get_new_id();
  VolumeMesh<double> deformable_mesh =
      MakeBoxVolumeMesh<double>(Box::MakeCube(1.0), 1.0);
  const int num_vertices = deformable_mesh.num_vertices();
  geometries.AddDeformableGeometry(deformable_id, std::move(deformable_mesh));
  collision_filter.AddGeometry(deformable_id);

  /* There is no geometry to collide with the deformable geometry yet. */
  contact_data = geometries.ComputeDeformableContact(collision_filter);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 0);
  /* Add a rigid unit cube. */
  GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties = MakeProximityPropsWithRezHint(1.0);
  math::RigidTransform<double> X_WR(Vector3d(0, -2.0, 0));
  geometries.MaybeAddRigidGeometry(Box::MakeCube(1.0), rigid_id,
                                   rigid_properties, X_WR);
  collision_filter.AddGeometry(rigid_id);

  /* The deformable box and the rigid box are not in contact yet. */
  contact_data = geometries.ComputeDeformableContact(collision_filter);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 0);

  /* Now shift the rigid geometry closer to the deformable geometry.
                                  +Z
                                   |
                                   |
             rigid box             |      deformable box
                   ----------+--+--+-------
                   |         |  ●  |      |
                   |         |  |  |      |
            -Y-----+---------+--+--+------+-------+Y
                   |         |  |  |      |
                   |         |  ●  |      |
                   ----------+--+--+-------
                                   |
                                   |
                                   |
                                  -Z
    where the "●"s denote representative contact points. */
  X_WR = math::RigidTransform<double>(Vector3d(0, -0.75, 0));
  geometries.UpdateRigidWorldPose(rigid_id, X_WR);

  /* Now there should be exactly one contact data. */
  contact_data = geometries.ComputeDeformableContact(collision_filter);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 1);

  /* Verify that the contact surface is as expected. */
  const auto& X_DR =
      X_WR;  // The deformable mesh frame is always the world frame.
  const DeformableGeometry& deformable_geometry =
      GeometriesTester::get_deformable_geometry(geometries, deformable_id);
  const RigidGeometry& rigid_geometry =
      GeometriesTester::get_rigid_geometry(geometries, rigid_id);
  DeformableContact<double> expected_contact_data;
  expected_contact_data.RegisterDeformableGeometry(deformable_id, num_vertices);
  AddDeformableRigidContactSurface(
      deformable_geometry.CalcSignedDistanceField(),
      deformable_geometry.deformable_mesh(), deformable_id, rigid_id,
      rigid_geometry.rigid_mesh().mesh(), rigid_geometry.rigid_mesh().bvh(),
      X_DR, &expected_contact_data);

  /* Verify that the contact data is the same as expected by checking a subset
   of all data fields. */
  ASSERT_EQ(contact_data.contact_surfaces().size(),
            expected_contact_data.contact_surfaces().size());
  const DeformableContactSurface<double>& contact_surface =
      contact_data.contact_surfaces()[0];
  const DeformableContactSurface<double>& expected_contact_surface =
      expected_contact_data.contact_surfaces()[0];
  // TODO(xuchenhan-tri): consider adding a `Equal` function for
  // DeformableContactSurface.
  EXPECT_EQ(contact_surface.id_A(), expected_contact_surface.id_A());
  EXPECT_EQ(contact_surface.id_B(), expected_contact_surface.id_B());
  EXPECT_EQ(contact_surface.num_contact_points(),
            expected_contact_surface.num_contact_points());
  EXPECT_TRUE(contact_surface.contact_mesh_W().Equal(
      expected_contact_surface.contact_mesh_W()));

  /* No contact is reported if the the pair of rigid and deformable geometries
   are filtered in the collision filter. */
  collision_filter.Apply(CollisionFilterDeclaration().ExcludeBetween(
                             GeometrySet(deformable_id), GeometrySet(rigid_id)),
                         get_extract_ids_functor());
  contact_data = geometries.ComputeDeformableContact(collision_filter);
  EXPECT_EQ(contact_data.contact_surfaces().size(), 0);
}

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
