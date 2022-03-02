#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;

enum class GeometryType { kRigid, kDeformable };

ProximityProperties MakeProximityPropsWithRezHint(GeometryType type,
                                                  double resolution_hint) {
  ProximityProperties props;
  props.AddProperty(internal::kHydroGroup, internal::kRezHint, resolution_hint);
  if (type == GeometryType::kRigid) {
    props.AddProperty(internal::kHydroGroup, internal::kComplianceType,
                      internal::HydroelasticType::kRigid);
  } else if (type == GeometryType::kDeformable) {
    props.AddProperty(internal::kHydroGroup, internal::kComplianceType,
                      internal::HydroelasticType::kSoft);
  } else {
    DRAKE_UNREACHABLE();
  }
  return props;
}

// TODO(xuchenhan-tri):Need to test signed distance field and other shapes than
// sphere.
GTEST_TEST(DeformableGeometryTest, MakeDeformableGeometry) {
  const ProximityProperties props =
      MakeProximityPropsWithRezHint(GeometryType::kDeformable, 0.5);
  const Sphere sphere(1.0);
  DeformableGeometry deformable_sphere =
      MakeDeformableRepresentation(sphere, props).value();

  const VolumeMesh<double>& mesh =
      deformable_sphere.deformable_volume_mesh().mesh();
  const int num_vertices = mesh.num_vertices();
  const VectorXd q = VectorXd::LinSpaced(3 * num_vertices, 0.0, 1.0);
  deformable_sphere.UpdateVertexPositions(q);

  for (int i = 0; i < num_vertices; ++i) {
    const Vector3d& q_MV = mesh.vertex(i);
    const Vector3d& expected_q_MV = q.segment<3>(3 * i);
    EXPECT_EQ(q_MV, expected_q_MV);
  }
}

GTEST_TEST(RigidGeometryTest, RigidGeometry) {
  const Sphere sphere(1.0);
  const double resolution_hint = 0.5;
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeSphereSurfaceMesh<double>(sphere, resolution_hint));
  auto rigid_mesh = make_unique<hydroelastic::RigidMesh>(std::move(mesh));
  RigidGeometry rigid_geometry(std::move(rigid_mesh));
  const math::RigidTransform<double> X_WG(
      math::RollPitchYaw<double>(-1.57, 0, 3), Vector3d(-0.3, -0.55, 0.36));
  rigid_geometry.set_pose_in_world(X_WG);
  EXPECT_TRUE(X_WG.IsExactlyEqualTo(rigid_geometry.pose_in_world()));
}

// Tests the simple public API of the deformable::Geometries: adding
// geometries and querying the data stored.
GTEST_TEST(DeformableContactTest, GeometriesPopulationAndQuery) {
  Geometries geometries;

  // Ids that haven't been added report as undefined.
  GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties =
      MakeProximityPropsWithRezHint(GeometryType::kRigid, 0.5);

  GeometryId deformable_id = GeometryId::get_new_id();
  ProximityProperties deformable_properties =
      MakeProximityPropsWithRezHint(GeometryType::kDeformable, 0.25);

  GeometryId bad_id = GeometryId::get_new_id();
  EXPECT_FALSE(geometries.is_rigid(rigid_id));
  EXPECT_FALSE(geometries.is_deformable(rigid_id));
  EXPECT_FALSE(geometries.is_rigid(deformable_id));
  EXPECT_FALSE(geometries.is_deformable(deformable_id));
  EXPECT_FALSE(geometries.is_rigid(bad_id));
  EXPECT_FALSE(geometries.is_deformable(bad_id));

  // Once added, they report the appropriate type.
  geometries.MaybeAddGeometry(Sphere(0.5), deformable_id,
                              deformable_properties);
  EXPECT_FALSE(geometries.is_rigid(deformable_id));
  EXPECT_TRUE(geometries.is_deformable(deformable_id));
  geometries.MaybeAddGeometry(Sphere(0.5), rigid_id, rigid_properties);
  EXPECT_TRUE(geometries.is_rigid(rigid_id));
  EXPECT_FALSE(geometries.is_deformable(rigid_id));
  // Ids that report the correct type, successfully access the appropriate
  // representation.
  DRAKE_EXPECT_NO_THROW(geometries.deformable_geometry(deformable_id));
  DRAKE_EXPECT_NO_THROW(geometries.rigid_geometry(rigid_id));
  // Ids that report the wrong type throw an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(geometries.rigid_geometry(deformable_id),
                              "There is no rigid geometry with GeometryId .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometries.deformable_geometry(rigid_id),
      "There is no deformable geometry with GeometryId .*");

  // Rigid geometry's pose can be updated.
  const math::RigidTransform<double> X_WG(
      math::RollPitchYaw<double>(-1.57, 0, 3), Vector3d(-0.3, -0.55, 0.36));
  geometries.UpdateRigidWorldPose(rigid_id, X_WG);
  const RigidGeometry& rigid_geometry = geometries.rigid_geometry(rigid_id);
  EXPECT_TRUE(rigid_geometry.pose_in_world().IsExactlyEqualTo(X_WG));
  // Deformable geometry's vertex positions can be updated.
  const DeformableGeometry& deformable_geometry =
      geometries.deformable_geometry(deformable_id);
  const int num_vertices =
      deformable_geometry.deformable_volume_mesh().mesh().num_vertices();
  const VectorXd q = VectorXd::LinSpaced(3 * num_vertices, 0.0, 1.0);
  geometries.UpdateDeformableVertexPositions(deformable_id, q);
  const VolumeMesh<double>& mesh =
      deformable_geometry.deformable_volume_mesh().mesh();
  for (int i = 0; i < num_vertices; ++i) {
    const Vector3d& q_MV = mesh.vertex(i);
    const Vector3d& expected_q_MV = q.segment<3>(3 * i);
    EXPECT_EQ(q_MV, expected_q_MV);
  }
}

GTEST_TEST(DeformableContactTest, RemoveGeometry) {
  Geometries geometries;

  GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties =
      MakeProximityPropsWithRezHint(GeometryType::kRigid, 0.5);
  geometries.MaybeAddGeometry(Sphere(0.5), rigid_id, rigid_properties);

  GeometryId deformable_id = GeometryId::get_new_id();
  ProximityProperties deformable_properties =
      MakeProximityPropsWithRezHint(GeometryType::kDeformable, 0.25);
  geometries.MaybeAddGeometry(Sphere(0.5), deformable_id,
                              deformable_properties);

  EXPECT_TRUE(geometries.is_rigid(rigid_id));
  EXPECT_TRUE(geometries.is_deformable(deformable_id));

  // Removing a non-existant geometry is a no-op.
  const GeometryId bad_id = GeometryId::get_new_id();
  EXPECT_NO_THROW(geometries.RemoveGeometry(bad_id));
  EXPECT_TRUE(geometries.is_rigid(rigid_id));
  EXPECT_TRUE(geometries.is_deformable(deformable_id));

  // Remove the deformable geometry.
  geometries.RemoveGeometry(deformable_id);
  EXPECT_TRUE(geometries.is_rigid(rigid_id));
  EXPECT_FALSE(geometries.is_deformable(deformable_id));

  // Remove the rigid geometry.
  geometries.RemoveGeometry(rigid_id);
  EXPECT_FALSE(geometries.is_rigid(rigid_id));
  EXPECT_FALSE(geometries.is_deformable(deformable_id));
}

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
