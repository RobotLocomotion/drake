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

/* Constructs a coarse sphere (octahedron) whose vertices are transformed from
 the canonical sphere frame S to some arbitrary frame F. */
VolumeMesh<double> MakeTransformedSphere(double radius,
                                         const math::RigidTransformd& X_FS) {
  const Sphere sphere = Sphere(radius);
  const VolumeMesh<double> mesh_S = MakeSphereVolumeMesh<double>(
      sphere, radius * 2, TessellationStrategy::kSingleInteriorVertex);
  const int num_vertices = mesh_S.num_vertices();

  std::vector<Vector3d> vertices_F;
  vertices_F.reserve(num_vertices);
  for (int v = 0; v < num_vertices; ++v) {
    vertices_F.push_back(X_FS * mesh_S.vertex(v));
  }
  std::vector<VolumeElement> tets_F(mesh_S.tetrahedra());
  return VolumeMesh<double>(move(tets_F), move(vertices_F));
}

GTEST_TEST(DeformableGeometryTest, Constructor) {
  constexpr double kRadius = 1;
  // This arbitrary transform (with the particularly oddly spelled rotation),
  // stresses the distance computation on the surface. Another transform may not
  // be so taxing.
  math::RigidTransformd X_WS(math::RotationMatrixd::MakeXRotation(0.5) *
                                 math::RotationMatrixd::MakeYRotation(0.3) *
                                 math::RotationMatrixd::MakeZRotation(0.9),
                             Vector3d(0.5, 0.7, 0.9));

  VolumeMesh<double> mesh_W = MakeTransformedSphere(kRadius, X_WS);
  // We leverage our knowledge about sphere mesh generation -- because the
  // the sphere is the coarsest possible, it is an octahedron with 8 tets and
  // 7 vertices (where the center vertex is at index 0).
  const int num_vertices = mesh_W.num_vertices();
  ASSERT_EQ(num_vertices, 7);
  const int kCenterVertexIndex = 0;

  DeformableGeometry deformable_geometry(mesh_W);

  auto verify_sdf = [num_vertices](const DeformableGeometry& geometry) {
    const VolumeMeshFieldLinear<double, double>& sdf =
        geometry.CalcSignedDistanceField();
    EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(kCenterVertexIndex),
                     -kRadius / std::sqrt(3));
    // Skipping center vertex = 0.
    for (int i = 1; i < num_vertices; ++i) {
      EXPECT_NEAR(sdf.EvaluateAtVertex(i), 0.0,
                  std::numeric_limits<double>::epsilon());
    }
  };
  verify_sdf(deformable_geometry);

  // Verify that the distance field is unaffected by deformation of the mesh.
  VectorXd q(3 * num_vertices);
  const double scale = 1.23;
  for (int v = 0; v < num_vertices; ++v) {
    q.segment<3>(3 * v) = scale * mesh_W.vertex(v);
  }
  deformable_geometry.UpdateVertexPositions(q);
  verify_sdf(deformable_geometry);
}

GTEST_TEST(DeformableGeometryTest, TestCopyAndMoveSemantics) {
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
    EXPECT_FALSE(
        copy.deformable_mesh().mesh().Equal(original.deformable_mesh().mesh()));
    copy = original;

    // Test for uniqueness.
    EXPECT_NE(&original.deformable_mesh(), &copy.deformable_mesh());
    EXPECT_NE(&original.deformable_mesh().mesh(),
              &copy.deformable_mesh().mesh());
    EXPECT_NE(&copy.deformable_mesh().bvh(), &original.deformable_mesh().bvh());

    EXPECT_TRUE(
        copy.deformable_mesh().mesh().Equal(original.deformable_mesh().mesh()));
    EXPECT_TRUE(
        copy.deformable_mesh().bvh().Equal(original.deformable_mesh().bvh()));

    const VolumeMeshFieldLinear<double, double>& copy_sdf =
        copy.CalcSignedDistanceField();
    const VolumeMeshFieldLinear<double, double>& original_sdf =
        original.CalcSignedDistanceField();
    EXPECT_TRUE(copy_sdf.Equal(original_sdf));
  }

  // Test copy constructor.
  {
    DeformableGeometry copy(original);

    // Test for uniqueness.
    EXPECT_NE(&original.deformable_mesh(), &copy.deformable_mesh());
    EXPECT_NE(&original.deformable_mesh().mesh(),
              &copy.deformable_mesh().mesh());
    EXPECT_NE(&copy.deformable_mesh().bvh(), &original.deformable_mesh().bvh());

    EXPECT_TRUE(
        copy.deformable_mesh().mesh().Equal(original.deformable_mesh().mesh()));
    EXPECT_TRUE(
        copy.deformable_mesh().bvh().Equal(original.deformable_mesh().bvh()));

    const VolumeMeshFieldLinear<double, double>& copy_sdf =
        copy.CalcSignedDistanceField();
    const VolumeMeshFieldLinear<double, double>& original_sdf =
        original.CalcSignedDistanceField();
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

    // Test move constructor.
    DeformableGeometry move_constructed(std::move(start));
    EXPECT_EQ(&move_constructed.deformable_mesh(), mesh_ptr);

    // Test move-assignment operator.
    DeformableGeometry move_assigned(dummy_mesh);
    move_assigned = std::move(move_constructed);
    EXPECT_EQ(&move_assigned.deformable_mesh(), mesh_ptr);
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
