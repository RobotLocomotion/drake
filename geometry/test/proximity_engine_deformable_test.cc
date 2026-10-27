#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/proximity/deformable_contact_internal.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace internal {

class ProximityEngineTester {
 public:
  ProximityEngineTester() = delete;

  template <typename T>
  static const internal::deformable::Geometries&
  get_deformable_contact_geometries(const ProximityEngine<T>& engine) {
    return engine.deformable_contact_geometries();
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

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using std::make_unique;

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

// As with hydro (see ProcessHydroelasticProperties test in
// proximity_engine_hydroelastic_test.cc), shapes are processed with one of two
// patterns: mesh-type or primitive (see the ProcessHydroelasticProperties
// test). In this case, we check for processing of deformable infrastructure on
// two representative shapes.
TEST_F(ProximityEngineDeformableContactTest, AddSupportedRigidGeometries) {
  ProximityProperties valid_props = MakeProximityPropsForRigidGeometry(1.0);
  ProximityProperties empty_props;
  // The primitive is a sphere, because not all shapes are supported.
  const Sphere sphere(0.5);
  TestRigidRegistrationAndRemoval(sphere, valid_props, true);
  TestRigidRegistrationAndRemoval(sphere, empty_props, false);

  const Mesh mesh(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"));
  TestRigidRegistrationAndRemoval(mesh, valid_props, true);
  TestRigidRegistrationAndRemoval(mesh, empty_props, false);
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
  // We're only sampling the first and last of evidence that the underlying
  // machinery has been invoked. Full verification of the math across the entire
  // mesh is done in the tests for deformable::Geometries.
  for (int i : {0, num_vertices - 1}) {
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

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
