#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/deformable_field_intersection.h"
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

  /* Expose the map for testing, so testers can use the order of GeometryId's in
     the map in a consistent way. For example, we will know that, for the two
     iterators it0 = deformable_geometries().begin() and
               it1 = std::next(it0, 1),
     we will have
               GeometryId deformable0_id = it0->first before
               GeometryId deformable1_id = it1->first  */
  static const std::unordered_map<GeometryId, DeformableGeometry>&
  deformable_geometries(const Geometries& geometries) {
    return geometries.deformable_geometries_;
  }

  static void disable_rigid_geometry_deferral(Geometries* geometries) {
    geometries->enable_rigid_geometries_pending_ = false;
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

void AddDeformableGeometry(GeometryId id, VolumeMesh<double> mesh,
                           Geometries* geometries) {
  std::vector<int> surface_vertices;
  std::vector<int> surface_tri_to_volume_tet;
  TriangleSurfaceMesh<double> surface_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh, &surface_vertices, &surface_tri_to_volume_tet);
  geometries->AddDeformableGeometry(
      id, std::move(mesh), std::move(surface_mesh), std::move(surface_vertices),
      std::move(surface_tri_to_volume_tet));
}

/* Makes a ProximityProperties with a resolution hint property in the hydro
 group.
 @pre resolution_hint > 0. */
ProximityProperties MakeCompliantHydroProps(double resolution_hint,
                                            double hydroelastic_modulus = 1e5) {
  ProximityProperties props;
  AddCompliantHydroelasticProperties(resolution_hint, hydroelastic_modulus,
                                     &props);
  props.AddProperty("hydroelastic", "slab_thickness", 0.1);
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
  ProximityProperties props = MakeCompliantHydroProps(kRezHint);
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), rigid_id, props,
                                   default_pose());

  EXPECT_TRUE(geometries.is_rigid(rigid_id));
  EXPECT_FALSE(geometries.is_deformable(rigid_id));

  /* Trying to add a rigid geometry without the resolution hint property is a
   no-op. */
  GeometryId g_id = GeometryId::get_new_id();
  ProximityProperties empty_props;
  geometries.MaybeAddRigidGeometry(Sphere(kRadius), g_id, empty_props,
                                   default_pose());

  EXPECT_FALSE(geometries.is_rigid(g_id));
  EXPECT_FALSE(geometries.is_deformable(g_id));

  /* Trying to add an unsupported rigid geometry is a no-op. */
  GeometryId half_space_id = GeometryId::get_new_id();
  geometries.MaybeAddRigidGeometry(HalfSpace{}, half_space_id, props,
                                   default_pose());
  EXPECT_FALSE(geometries.is_rigid(half_space_id));
  EXPECT_FALSE(geometries.is_deformable(half_space_id));
}

/* Test coverage for all unsupported shapes as rigid geometries: MeshcatCone and
 HalfSpace. */
GTEST_TEST(GeometriesTest, UnsupportedRigidShapes) {
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeCompliantHydroProps(kRezHint);
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
  ProximityProperties props = MakeCompliantHydroProps(kRezHint);
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
  GeometriesTester::disable_rigid_geometry_deferral(&geometries);

  /* Add a rigid geometry. */
  GeometryId rigid_id = GeometryId::get_new_id();
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeCompliantHydroProps(kRezHint);
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
  AddDeformableGeometry(deformable_id, MakeVolumeMesh(), &geometries);
  EXPECT_FALSE(geometries.is_rigid(deformable_id));
  EXPECT_TRUE(geometries.is_deformable(deformable_id));
}

GTEST_TEST(GeometriesTest, RemoveGeometry) {
  Geometries geometries;
  /* Add a couple of deformable geometries. */
  GeometryId deformable_id0 = GeometryId::get_new_id();
  GeometryId deformable_id1 = GeometryId::get_new_id();
  AddDeformableGeometry(deformable_id0, MakeVolumeMesh(), &geometries);
  AddDeformableGeometry(deformable_id1, MakeVolumeMesh(), &geometries);

  /* Add a couple of rigid geometries. */
  GeometryId rigid_id0 = GeometryId::get_new_id();
  GeometryId rigid_id1 = GeometryId::get_new_id();
  constexpr double kRadius = 0.5;
  constexpr double kRezHint = 0.5;
  ProximityProperties props = MakeCompliantHydroProps(kRezHint);
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
  const TriangleSurfaceMesh<double> input_surface_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(input_mesh);

  AddDeformableGeometry(deformable_id, input_mesh, &geometries);
  const int num_vertices = input_mesh.num_vertices();
  const int num_surface_vertices = input_surface_mesh.num_vertices();

  /* Initially the vertex positions is the same as the registered mesh. */
  {
    ASSERT_TRUE(geometries.is_deformable(deformable_id));
    const DeformableGeometry& geometry =
        GeometriesTester::get_deformable_geometry(geometries, deformable_id);
    EXPECT_TRUE(geometry.deformable_volume().mesh().Equal(input_mesh));
  }
  /* Update the vertex positions to some arbitrary value. */
  const VectorXd q = VectorXd::LinSpaced(3 * num_vertices, 0.0, 1.0);
  const VectorXd q_surface =
      VectorXd::LinSpaced(3 * num_surface_vertices, 0.0, 1.0);
  geometries.UpdateDeformableVertexPositions(deformable_id, q, q_surface);
  {
    const DeformableGeometry& geometry =
        GeometriesTester::get_deformable_geometry(geometries, deformable_id);
    const VolumeMesh<double>& mesh = geometry.deformable_volume().mesh();
    for (int i = 0; i < num_vertices; ++i) {
      const Vector3d& q_MV = mesh.vertex(i);
      const Vector3d& reference_q_MV = input_mesh.vertex(i);
      const Vector3d& expected_q_MV = q.segment<3>(3 * i);
      EXPECT_EQ(q_MV, expected_q_MV);
      EXPECT_NE(q_MV, reference_q_MV);
    }
    const TriangleSurfaceMesh<double>& surface_mesh =
        geometry.deformable_surface().mesh();
    for (int i = 0; i < num_surface_vertices; ++i) {
      const Vector3d& q_MV = surface_mesh.vertex(i);
      const Vector3d& reference_q_MV = input_surface_mesh.vertex(i);
      const Vector3d& expected_q_MV = q_surface.segment<3>(3 * i);
      EXPECT_EQ(q_MV, expected_q_MV);
      EXPECT_NE(q_MV, reference_q_MV);
    }
  }
}

// This test focuses on the contact between a deformable geometry and a rigid
// geometry. The next test will have contacts between deformable geometries.
GTEST_TEST(GeometriesTest, ComputeDeformableContact_DeformableRigid) {
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
  AddDeformableGeometry(deformable_id, std::move(deformable_mesh), &geometries);
  collision_filter.AddGeometry(deformable_id);

  /* There is no geometry to collide with the deformable geometry yet. */
  contact_data = geometries.ComputeDeformableContact(collision_filter);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 0);
  /* Add a rigid unit cube. */
  GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties = MakeCompliantHydroProps(1.0);
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
                   |         |  |  |      |
                   |         ●  |  |      |
            -Y-----+---------+--+--+------+-------+Y
                   |         |  |  |      |
                   |         ●  |  |      |
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
  const auto X_RD = X_DR.inverse();
  const DeformableGeometry& deformable_geometry =
      GeometriesTester::get_deformable_geometry(geometries, deformable_id);
  const RigidGeometry& rigid_geometry =
      GeometriesTester::get_rigid_geometry(geometries, rigid_id);
  const VolumeMeshFieldLinear<double, double>& pressure_field =
      rigid_geometry.mesh().pressure();
  DeformableContact<double> expected_contact_data;
  expected_contact_data.RegisterDeformableGeometry(deformable_id, num_vertices);
  AddDeformableRigidContactSurface(
      deformable_geometry.deformable_surface(),
      deformable_geometry.deformable_volume(),
      deformable_geometry.surface_index_to_volume_index(),
      deformable_geometry.surface_tri_to_volume_tet(), deformable_id, rigid_id,
      pressure_field, rigid_geometry.mesh().bvh(), X_RD,
      &expected_contact_data);

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

// Test contact between deformable geometries. It is separated from the
// previous deformable-rigid contact for ease of maintenance. The main
// objectives are
//   1. For one contact pair, ComputeDeformableContact() passes arguments to
//      AddDeformableDeformableContactSurface() in the right order.
//   2. For n deformable geometries, ComputeDeformableContact() gives
//      n(n-1)/2 unique pairs of contacts.
//   3. ComputeDeformableContact() respect the collision filter.
class DeformableDeformableContactTest : public ::testing::Test {
 protected:
  // Set up two deformable geometries in contact.
  void SetUp() override {
    collision_filter_.AddGeometry(deformable0_id_);
    collision_filter_.AddGeometry(deformable1_id_);

    // Use Box meshes with medial axis because they have no zero interpolated
    // signed distance in the interior volume. For simplicity, we use two long
    // boxes aligned with different axes.
    //                    Z
    //                   | |
    //                   | |
    //                   | |
    //        -----------| |------------
    //        -----------| |------------  X
    //                   | |
    //                   | |
    //                   | |
    //
    VolumeMesh<double> mesh0 =
        MakeBoxVolumeMeshWithMa<double>(Box(0.3, 0.01, 0.01));
    VolumeMesh<double> mesh1 =
        MakeBoxVolumeMeshWithMa<double>(Box(0.01, 0.01, 0.3));

    AddDeformableGeometry(deformable0_id_, std::move(mesh0), &geometries_);
    AddDeformableGeometry(deformable1_id_, std::move(mesh1), &geometries_);
  }

  const GeometryId deformable0_id_{GeometryId::get_new_id()};
  const GeometryId deformable1_id_{GeometryId::get_new_id()};
  Geometries geometries_;
  CollisionFilter collision_filter_;
};

// For one contact pair, ComputeDeformableContact() passes arguments to
// AddDeformableDeformableContactSurface() in the right order.
TEST_F(DeformableDeformableContactTest, OneContactPair) {
  DeformableContact<double> contact_data =
      geometries_.ComputeDeformableContact(collision_filter_);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 1);

  DeformableContact<double> expected;
  const DeformableGeometry& deformable0_geometry =
      GeometriesTester::get_deformable_geometry(geometries_, deformable0_id_);
  const DeformableGeometry& deformable1_geometry =
      GeometriesTester::get_deformable_geometry(geometries_, deformable1_id_);
  expected.RegisterDeformableGeometry(
      deformable0_id_,
      deformable0_geometry.deformable_volume().mesh().num_vertices());
  expected.RegisterDeformableGeometry(
      deformable1_id_,
      deformable1_geometry.deformable_volume().mesh().num_vertices());
  AddDeformableDeformableContactSurface(
      deformable1_geometry.CalcSignedDistanceField(),
      deformable1_geometry.deformable_volume(), deformable1_id_,
      deformable0_geometry.CalcSignedDistanceField(),
      deformable0_geometry.deformable_volume(), deformable0_id_, &expected);
  ASSERT_EQ(expected.contact_surfaces().size(), 1);

  ASSERT_TRUE(contact_data.contact_surfaces().at(0).id_A() == deformable0_id_ &&
              contact_data.contact_surfaces().at(0).id_B() == deformable1_id_);

  ASSERT_EQ(contact_data.contact_surfaces().size(),
            expected.contact_surfaces().size());
  EXPECT_EQ(contact_data.contact_surfaces().at(0).id_A(),
            expected.contact_surfaces().at(0).id_A());
  EXPECT_EQ(contact_data.contact_surfaces().at(0).id_B(),
            expected.contact_surfaces().at(0).id_B());
  EXPECT_EQ(contact_data.contact_surfaces().at(0).num_contact_points(),
            expected.contact_surfaces().at(0).num_contact_points());
  EXPECT_TRUE(contact_data.contact_surfaces().at(0).contact_mesh_W().Equal(
      expected.contact_surfaces().at(0).contact_mesh_W()));
}

// For n deformable geometries, ComputeDeformableContact() gives
// n(n-1)/2 unique pairs of contacts. If it creates
// DeformableContactSurface(GeometryId_i, GeometryId_j), i≠j, it does
// not create the "duplicated" DeformableContactSurface(GeometryId_j,
// GeometryId_i).
TEST_F(DeformableDeformableContactTest, MultipleContactPairs) {
  // We will test with n=3 by adding the third deformable box.
  //
  //                    Z     Y
  //                   | |  / /
  //                   | | / /
  //                   | |/ /
  //                   | / /
  //        -----------|/|/----------
  //        -----------/ /|----------  X
  //                  /|/|
  //                 / / |
  //                / /| |
  //               / / | |
  //
  {
    const GeometryId deformable2_id = GeometryId::get_new_id();
    collision_filter_.AddGeometry(deformable2_id);
    VolumeMesh<double> mesh2 =
        MakeBoxVolumeMeshWithMa<double>(Box(0.01, 0.3, 0.01));
    AddDeformableGeometry(deformable2_id, std::move(mesh2), &geometries_);
  }
  ASSERT_EQ(GeometriesTester::deformable_geometries(geometries_).size(), 3);
  auto it = GeometriesTester::deformable_geometries(geometries_).begin();
  const GeometryId deformable0_id = it->first;
  const GeometryId deformable1_id = (++it)->first;
  const GeometryId deformable2_id = (++it)->first;

  DeformableContact<double> contact_data =
      geometries_.ComputeDeformableContact(collision_filter_);
  EXPECT_EQ(contact_data.contact_surfaces().size(), 3);

  // Verify unique pairs of contact without duplication.
  std::set<std::set<GeometryId>> expected_pairs{
      {deformable0_id, deformable1_id},
      {deformable0_id, deformable2_id},
      {deformable1_id, deformable2_id}};
  std::set<std::set<GeometryId>> pairs{
      {contact_data.contact_surfaces().at(0).id_A(),
       contact_data.contact_surfaces().at(0).id_B()},
      {contact_data.contact_surfaces().at(1).id_A(),
       contact_data.contact_surfaces().at(1).id_B()},
      {contact_data.contact_surfaces().at(2).id_A(),
       contact_data.contact_surfaces().at(2).id_B()}};
  EXPECT_EQ(pairs, expected_pairs);
}

// ComputeDeformableContact() respects the collision filter.
TEST_F(DeformableDeformableContactTest, RespectCollisionFilter) {
  // First, we have one contact patch.
  DeformableContact<double> contact_data =
      geometries_.ComputeDeformableContact(collision_filter_);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 1);

  // Then, we use collision filter to disable contacts.
  SCOPED_TRACE("Collision filter");
  ASSERT_EQ(GeometriesTester::deformable_geometries(geometries_).size(), 2);
  auto it = GeometriesTester::deformable_geometries(geometries_).begin();
  const GeometryId deformable0_id = it->first;
  const GeometryId deformable1_id = (++it)->first;
  collision_filter_.Apply(CollisionFilterDeclaration().ExcludeWithin(
                              GeometrySet({deformable0_id, deformable1_id})),
                          get_extract_ids_functor());

  contact_data = geometries_.ComputeDeformableContact(collision_filter_);
  EXPECT_EQ(contact_data.contact_surfaces().size(), 0);
}

GTEST_TEST(GeometriesTest, GetDeformableAabbInWorld) {
  Geometries geometries;

  // Create a deformable sphere with radius 0.5.
  const GeometryId deformable_id = GeometryId::get_new_id();
  const VolumeMesh<double> mesh = MakeVolumeMesh();
  const TriangleSurfaceMesh<double> surface_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(mesh);
  const int num_vertices = mesh.num_vertices();
  const int num_surface_vertices = surface_mesh.num_vertices();
  AddDeformableGeometry(deformable_id, mesh, &geometries);

  // The AABB should be a unit box centered at the origin.
  const Aabb& aabb = geometries.GetDeformableAabbInWorld(deformable_id);
  EXPECT_TRUE(CompareMatrices(aabb.lower(), Vector3d(-0.5, -0.5, -0.5)));
  EXPECT_TRUE(CompareMatrices(aabb.upper(), Vector3d(0.5, 0.5, 0.5)));

  // Update the vertex positions so that the sphere is shifted to the right.
  const Vector3d p_B0B1_W = Vector3d(1.0, 0.0, 0.0);
  VectorXd q = VectorXd::Zero(3 * num_vertices);
  VectorXd q_surface = VectorXd::Zero(3 * num_surface_vertices);
  for (int i = 0; i < num_vertices; ++i) {
    q.segment<3>(3 * i) = mesh.vertex(i) + p_B0B1_W;
  }
  for (int i = 0; i < num_surface_vertices; ++i) {
    q_surface.segment<3>(3 * i) = surface_mesh.vertex(i) + p_B0B1_W;
  }
  geometries.UpdateDeformableVertexPositions(deformable_id, q, q_surface);

  // The AABB should be a unit box centered at (1, 0, 0).
  const Aabb& aabb_new = geometries.GetDeformableAabbInWorld(deformable_id);
  EXPECT_TRUE(CompareMatrices(aabb_new.lower(), Vector3d(0.5, -0.5, -0.5)));
  EXPECT_TRUE(CompareMatrices(aabb_new.upper(), Vector3d(1.5, 0.5, 0.5)));
}

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
