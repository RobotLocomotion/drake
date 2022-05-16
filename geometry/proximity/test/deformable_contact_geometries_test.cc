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

GTEST_TEST(ReferenceDeformableGeometryTest, TestCopyAssignConstruct) {
  const Sphere sphere(1.0);
  const double resolution_hint = 0.5;
  VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(
      sphere, resolution_hint, TessellationStrategy::kSingleInteriorVertex);

  const ReferenceDeformableGeometry original(sphere, mesh);

  // Test copy-assignment operator.
  {
    /* There is no default constructor so we initialize copy to be different
     from `original`. */
    const Sphere sphere2(2.0);
    const double resolution_hint2 = 1.0;
    VolumeMesh<double> mesh2 = MakeSphereVolumeMesh<double>(
        sphere2, resolution_hint2, TessellationStrategy::kSingleInteriorVertex);
    ReferenceDeformableGeometry copy(sphere2, mesh2);

    copy = original;
    // Test for uniqueness.
    EXPECT_NE(&original.signed_distance_field(), &copy.signed_distance_field());

    const VolumeMeshFieldLinear<double, double>& copy_sdf =
        copy.signed_distance_field();
    const VolumeMeshFieldLinear<double, double>& original_sdf =
        original.signed_distance_field();
    EXPECT_TRUE(copy_sdf.Equal(original_sdf));
  }

  // Test copy constructor.
  {
    ReferenceDeformableGeometry copy(original);
    // Test for uniqueness.
    EXPECT_NE(&original.signed_distance_field(), &copy.signed_distance_field());

    const VolumeMeshFieldLinear<double, double>& copy_sdf =
        copy.signed_distance_field();
    const VolumeMeshFieldLinear<double, double>& original_sdf =
        original.signed_distance_field();
    EXPECT_TRUE(copy_sdf.Equal(original_sdf));
  }
}

GTEST_TEST(ReferenceDeformableGeometryTest, Sphere) {
  const Sphere sphere(1.0);
  VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(
      sphere, 100 /* ensure we get the coarsest possible mesh. */,
      TessellationStrategy::kDenseInteriorVertices);
  const int num_vertices = mesh.num_vertices();
  // We leverage our knowledge about sphere mesh generation -- the targeted edge
  // length is set to the coarsest possible, so the generated mesh is an
  // octahedron. So we expect 7 vertices: 6 on the surface of the sphere and 1
  // at the center.
  ASSERT_EQ(num_vertices, 7);
  ReferenceDeformableGeometry reference_geometry(sphere, move(mesh));
  const VolumeMeshFieldLinear<double, double>& sdf =
      reference_geometry.signed_distance_field();
  EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(0), -1.0);
  for (int i = 1; i < num_vertices; ++i) {
    EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(i), 0.0);
  }
}

GTEST_TEST(ReferenceDeformableGeometryTest, Box) {
  const Box box = Box::MakeCube(1.0);
  VolumeMesh<double> mesh = MakeBoxVolumeMesh<double>(box, 0.5);
  const int num_vertices = mesh.num_vertices();
  // We leverage our knowledge about box mesh generation -- the targeted edge
  // length is set to half the edge length, so we expect 8 subcubes to be
  // created with a total of 27 vertices.
  ASSERT_EQ(num_vertices, 27);
  const int center_vertex_index = 13;

  ReferenceDeformableGeometry reference_geometry(box, move(mesh));
  const VolumeMeshFieldLinear<double, double>& sdf =
      reference_geometry.signed_distance_field();
  EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(center_vertex_index), -0.5);
  for (int i = 0; i < num_vertices; ++i) {
    if (i == center_vertex_index) continue;
    EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(i), 0.0);
  }
}

GTEST_TEST(DeformableGeometryTest, Constructor) {
  const Sphere sphere(1.0);
  VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(
      sphere, 0.5, TessellationStrategy::kDenseInteriorVertices);
  ReferenceDeformableGeometry reference_geometry(sphere, mesh);
  DeformableGeometry deformable_geometry(sphere, mesh);

  EXPECT_TRUE(deformable_geometry.signed_distance_field().Equal(
      reference_geometry.signed_distance_field()));
  EXPECT_TRUE(deformable_geometry.deformable_mesh().mesh().Equal(mesh));
}

GTEST_TEST(DeformableGeometryTest, UpdateVertexPositions) {
  const Sphere sphere(1.0);
  VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(
      sphere, 0.5, TessellationStrategy::kDenseInteriorVertices);
  const int num_vertices = mesh.num_vertices();
  DeformableGeometry deformable_geometry(sphere, move(mesh));
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
