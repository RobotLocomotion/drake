#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

class GeometriesTester {
 public:
  /* Returns the deformable geometry with `deformable_id` registered in
   `geometries`.
   @pre a deformable representatino with `deformable_id` exists in `geometries`.
  */
  static const DeformableGeometry& get_deformable_geometry(
      const Geometries& geometries, GeometryId deformable_id) {
    return geometries.deformable_geometries_.at(deformable_id);
  }

  /* Returns the rigid geometry with `rigid_id` registered in `geometries`.
   @pre a rigid representatino with `rigid_id` exists in `geometries`. */
  static const RigidGeometry& get_rigid_geometry(const Geometries& geometries,
                                                 GeometryId rigid_id) {
    return geometries.rigid_geometries_.at(rigid_id);
  }
};

namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

enum class GeometryType { kRigid, kDeformable };

/* Makes a ProximityProperties with two properties in the hydro group.
 1. A compliance type property that is set to `kSoft` if the input `type` is
   `kDeformable` and to `kRigid` otherwise.
 2. A resolution hint property set to the given resolution hint. */
ProximityProperties MakeProximityPropsWithRezHint(GeometryType type,
                                                  double resolution_hint) {
  ProximityProperties props;
  props.AddProperty(internal::kHydroGroup, internal::kRezHint, resolution_hint);
  if (type == GeometryType::kRigid) {
    props.AddProperty(internal::kHydroGroup, internal::kComplianceType,
                      internal::HydroelasticType::kRigid);
  } else if (type == GeometryType::kDeformable) {
    props.AddProperty(internal::kHydroGroup, internal::kComplianceType,
                      internal::HydroelasticType::kDeformable);
  } else {
    DRAKE_UNREACHABLE();
  }
  return props;
}

// Tests the simple public API of the deformable::Geometries: adding
// geometries and querying the data stored.
GTEST_TEST(DeformableContactInternalTest, GeometriesPopulationAndQuery) {
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

GTEST_TEST(DeformableContactInternalTest, RemoveGeometry) {
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

GTEST_TEST(DeformableContactInternalTest, CalcDeformableContactData) {
  Geometries geometries;
  // The contact data is empty when there is no deformable geometry.
  std::vector<DeformableContactData<double>> contact_data;
  geometries.ComputeAllDeformableContactData(&contact_data);
  EXPECT_EQ(contact_data.size(), 0);

  // Add a deformable unit cube.
  GeometryId deformable_id = GeometryId::get_new_id();
  ProximityProperties deformable_properties =
      MakeProximityPropsWithRezHint(GeometryType::kDeformable, 1.0);
  geometries.MaybeAddGeometry(Box(1.0, 1.0, 1.0), deformable_id,
                              deformable_properties);

  // There is no rigid geometry to collide with the deformable geometry yet.
  geometries.ComputeAllDeformableContactData(&contact_data);
  ASSERT_EQ(contact_data.size(), 1);
  EXPECT_EQ(contact_data[0].deformable_geometry_id(), deformable_id);
  EXPECT_EQ(contact_data[0].num_contact_pairs(), 0);
  EXPECT_EQ(contact_data[0].num_contact_points(), 0);
  EXPECT_EQ(contact_data[0].num_vertices_in_contact(), 0);

  // Add a rigid unit cube.
  GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties =
      MakeProximityPropsWithRezHint(GeometryType::kRigid, 1.0);
  geometries.MaybeAddGeometry(Box(1.0, 1.0, 1.0), rigid_id, rigid_properties);
  math::RigidTransform<double> X_WR(Vector3d(0, -2.0, 0));
  geometries.UpdateRigidWorldPose(rigid_id, X_WR);

  // The deformable box and the rigid box are not in contact yet.
  geometries.ComputeAllDeformableContactData(&contact_data);
  ASSERT_EQ(contact_data.size(), 1);
  EXPECT_EQ(contact_data[0].deformable_geometry_id(), deformable_id);
  EXPECT_EQ(contact_data[0].num_contact_pairs(), 0);
  EXPECT_EQ(contact_data[0].num_contact_points(), 0);
  EXPECT_EQ(contact_data[0].num_vertices_in_contact(), 0);

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

  // Now there should be exactly one contact pair.
  geometries.ComputeAllDeformableContactData(&contact_data);
  ASSERT_EQ(contact_data.size(), 1);
  const DeformableContactData<double>& box_box_contact_data = contact_data[0];
  ASSERT_EQ(box_box_contact_data.num_contact_pairs(), 1);
  EXPECT_EQ(box_box_contact_data.deformable_geometry_id(), deformable_id);
  const DeformableRigidContactPair<double> contact_pair =
      box_box_contact_data.contact_pairs()[0];

  // Verify that the contact surface is as expected.
  const auto& X_DR =
      X_WR;  // The deformable mesh frame is always the world frame.
  const DeformableGeometry& deformable_geometry =
      GeometriesTester::get_deformable_geometry(geometries, deformable_id);
  const RigidGeometry& rigid_geometry =
      GeometriesTester::get_rigid_geometry(geometries, rigid_id);
  const DeformableContactSurface<double> expected_contact_surface =
      ComputeTetMeshTriMeshContact<double>(
          deformable_geometry.deformable_volume_mesh(),
          rigid_geometry.rigid_mesh().mesh(), rigid_geometry.rigid_mesh().bvh(),
          X_DR);
  EXPECT_EQ(contact_pair.num_contact_points(),
            expected_contact_surface.num_polygons());
  const int num_contacts = expected_contact_surface.num_polygons();
  for (int i = 0; i < num_contacts; ++i) {
    const auto& expected_polygon_data =
        expected_contact_surface.polygon_data(i);
    const auto& calculated_polygon_data =
        contact_pair.contact_surface.polygon_data(i);
    EXPECT_EQ(expected_polygon_data.area, calculated_polygon_data.area);
    EXPECT_TRUE(CompareMatrices(expected_polygon_data.unit_normal,
                                calculated_polygon_data.unit_normal));
    EXPECT_TRUE(CompareMatrices(expected_polygon_data.centroid,
                                calculated_polygon_data.centroid));
    EXPECT_TRUE(CompareMatrices(expected_polygon_data.b_centroid,
                                calculated_polygon_data.b_centroid));
    EXPECT_EQ(expected_polygon_data.tet_index,
              calculated_polygon_data.tet_index);
  }

  // Verify the calculated rotation matrices map contact normals from
  // world frame to contact frame ({0,0,1}).
  constexpr double kTol = std::numeric_limits<double>::epsilon();
  ASSERT_EQ(contact_pair.R_CWs.size(), num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    EXPECT_TRUE(CompareMatrices(
        contact_pair.R_CWs[i] *
            contact_pair.contact_surface.polygon_data(i).unit_normal,
        Vector3d(0, 0, 1), kTol));
  }
}

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
