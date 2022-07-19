#include "drake/geometry/proximity/deformable_contact_geometries.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::move;

GTEST_TEST(DeformableGeometryTest, Constructor) {
  constexpr double kEdgeLength = 1.0;
  const Box box = Box::MakeCube(kEdgeLength);
  constexpr double kRezHint = 0.5;
  VolumeMesh<double> mesh = MakeBoxVolumeMesh<double>(box, kRezHint);
  const int num_vertices = mesh.num_vertices();
  // We leverage our knowledge about box mesh generation -- the targeted edge
  // length is set to half the edge length, so we expect 8 subcubes to be
  // created with a total of 27 vertices.
  ASSERT_EQ(num_vertices, 27);
  const int center_vertex_index = 13;

  DeformableGeometry deformable_geometry(mesh);

  auto verify_sdf = [num_vertices](const DeformableGeometry& geometry) {
    const VolumeMeshFieldLinear<double, double>& sdf =
        geometry.signed_distance_field();
    EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(center_vertex_index),
                     -0.5 * kEdgeLength);
    for (int i = 0; i < num_vertices; ++i) {
      if (i == center_vertex_index) continue;
      EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(i), 0.0);
    }
  };
  verify_sdf(deformable_geometry);

  // Verify that the distance field is unaffected by deformation of the mesh.
  const VectorXd q = VectorXd::LinSpaced(3 * num_vertices, 0.0, 1.0);
  deformable_geometry.UpdateVertexPositions(q);
  verify_sdf(deformable_geometry);
}

GTEST_TEST(DeformableGeometryTest, TestCopyMoveAssignConstruct) {
  constexpr double kEdgeLength = 1.0;
  const Box box = Box::MakeCube(kEdgeLength);
  const double kRezHint = 0.5;
  VolumeMesh<double> mesh = MakeBoxVolumeMesh<double>(box, kRezHint);
  DeformableGeometry original(mesh);

  std::vector<Vector3d> dummy_vertices = {Vector3d(0, 0, 0), Vector3d(1, 0, 0),
                                          Vector3d(0, 1, 0), Vector3d(0, 0, 1)};
  std::vector<VolumeElement> dummy_elements = {VolumeElement(0, 1, 2, 3)};
  VolumeMesh dummy_mesh(std::move(dummy_elements), std::move(dummy_vertices));

  // Test copy-assignment operator.
  {
    DeformableGeometry copy(dummy_mesh);
    copy = original;

    // Test for uniqueness.
    EXPECT_NE(&original.deformable_mesh(), &copy.deformable_mesh());
    EXPECT_NE(&original.signed_distance_field(), &copy.signed_distance_field());

    EXPECT_TRUE(
        copy.deformable_mesh().mesh().Equal(original.deformable_mesh().mesh()));

    const VolumeMeshFieldLinear<double, double>& copy_sdf =
        copy.signed_distance_field();
    const VolumeMeshFieldLinear<double, double>& original_sdf =
        original.signed_distance_field();
    EXPECT_TRUE(copy_sdf.Equal(original_sdf));
  }

  // Test copy constructor.
  {
    DeformableGeometry copy(original);

    // Test for uniqueness.
    EXPECT_NE(&original.deformable_mesh(), &copy.deformable_mesh());
    EXPECT_NE(&original.signed_distance_field(), &copy.signed_distance_field());

    EXPECT_TRUE(
        copy.deformable_mesh().mesh().Equal(original.deformable_mesh().mesh()));

    const VolumeMeshFieldLinear<double, double>& copy_sdf =
        copy.signed_distance_field();
    const VolumeMeshFieldLinear<double, double>& original_sdf =
        original.signed_distance_field();
    EXPECT_TRUE(copy_sdf.Equal(original_sdf));
  }

  // Test move constructor and move-assignment operator.
  // We will move the content from `start` to `move_constructed` to
  // `move_assigned`, each time confirming that the target of the move has taken
  // ownership.
  {
    DeformableGeometry start(
        original);  // Assume the copy constructor is correct.

    // Grab raw pointers so we can determine that their ownership changes due to
    // move semantics.
    const DeformableVolumeMesh<double>* const mesh_ptr =
        &start.deformable_mesh();
    const VolumeMeshFieldLinear<double, double>* const sdf_ptr =
        &start.signed_distance_field();

    // Test move constructor.
    DeformableGeometry move_constructed(std::move(start));
    EXPECT_EQ(&move_constructed.deformable_mesh(), mesh_ptr);
    EXPECT_EQ(&move_constructed.signed_distance_field(), sdf_ptr);

    // Test move-assignment operator.
    DeformableGeometry move_assigned(dummy_mesh);
    move_assigned = std::move(move_constructed);
    EXPECT_EQ(&move_assigned.deformable_mesh(), mesh_ptr);
    EXPECT_EQ(&move_assigned.signed_distance_field(), sdf_ptr);
  }
}

GTEST_TEST(DeformableGeometryTest, UpdateVertexPositions) {
  const Sphere sphere(1.0);
  VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(
      sphere, 0.5, TessellationStrategy::kDenseInteriorVertices);
  const int num_vertices = mesh.num_vertices();
  DeformableGeometry deformable_geometry(move(mesh));
  const VectorXd q = VectorXd::LinSpaced(3 * num_vertices, 0.0, 1.0);
  deformable_geometry.UpdateVertexPositions(q);
  const VolumeMesh<double> deformed_mesh =
      deformable_geometry.deformable_mesh().mesh();
  for (int i = 0; i < num_vertices; ++i) {
    const Vector3d& q_MV = deformed_mesh.vertex(i);
    const Vector3d& expected_q_MV = q.segment<3>(3 * i);
    EXPECT_EQ(q_MV, expected_q_MV);
  }
}

GTEST_TEST(RigidGeometryTest, Pose) {
  const Sphere sphere(1.0);
  const double resolution_hint = 0.5;
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeSphereSurfaceMesh<double>(sphere, resolution_hint));
  auto rigid_mesh = make_unique<hydroelastic::RigidMesh>(move(mesh));
  RigidGeometry rigid_geometry(move(rigid_mesh));
  const math::RigidTransform<double> X_WG(
      math::RollPitchYaw<double>(-1.57, 0, 3), Vector3d(-0.3, -0.55, 0.36));
  rigid_geometry.set_pose_in_world(X_WG);
  EXPECT_TRUE(X_WG.IsExactlyEqualTo(rigid_geometry.pose_in_world()));
}

GTEST_TEST(RigidGeometryTest, MakeRigidRepresentation) {
  const Sphere sphere(1.0);
  ProximityProperties props;
  const double resolution_hint = 0.5;
  AddRigidHydroelasticProperties(resolution_hint, &props);
  DRAKE_EXPECT_NO_THROW(MakeRigidRepresentation(sphere, props));

  const HalfSpace half_space;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeRigidRepresentation(half_space, props),
                              "Half space.*not.*supported.*");
}

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
