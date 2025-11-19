#include "drake/geometry/proximity/deformable_contact_geometries.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
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
  return VolumeMesh<double>(std::move(tets_F), std::move(vertices_F));
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

  std::vector<int> surface_vertices;
  std::vector<int> surface_tri_to_volume_tet;
  TriangleSurfaceMesh<double> surface_mesh_W =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh_W, &surface_vertices, &surface_tri_to_volume_tet);
  const int num_surface_vertices = surface_mesh_W.num_vertices();
  DeformableGeometry deformable_geometry(
      mesh_W, surface_mesh_W, surface_vertices, surface_tri_to_volume_tet);

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
  VectorXd q_surface(3 * num_surface_vertices);
  const double scale = 1.23;
  for (int v = 0; v < num_vertices; ++v) {
    q.segment<3>(3 * v) = scale * mesh_W.vertex(v);
  }
  for (int v = 0; v < num_surface_vertices; ++v) {
    q_surface.segment<3>(3 * v) = scale * surface_mesh_W.vertex(v);
  }
  deformable_geometry.UpdateVertexPositions(q, q_surface);
  verify_sdf(deformable_geometry);
}

GTEST_TEST(DeformableGeometryTest, TestCopyAndMoveSemantics) {
  constexpr double kEdgeLength = 1.0;
  const Box box = Box::MakeCube(kEdgeLength);
  const double kRezHint = 0.5;
  VolumeMesh<double> mesh = MakeBoxVolumeMesh<double>(box, kRezHint);
  std::vector<int> surface_vertices;
  std::vector<int> surface_tri_to_volume_tet;
  TriangleSurfaceMesh<double> surface_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh, &surface_vertices, &surface_tri_to_volume_tet);
  DeformableGeometry original(mesh, surface_mesh, surface_vertices,
                              surface_tri_to_volume_tet);

  std::vector<Vector3d> dummy_vertices = {Vector3d(0, 0, 0), Vector3d(1, 0, 0),
                                          Vector3d(0, 1, 0), Vector3d(0, 0, 1)};
  std::vector<VolumeElement> dummy_elements = {VolumeElement(0, 1, 2, 3)};
  VolumeMesh dummy_mesh(std::move(dummy_elements), std::move(dummy_vertices));
  dummy_vertices = {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0)};
  std::vector<SurfaceTriangle> dummy_triangles = {SurfaceTriangle(0, 1, 2)};
  TriangleSurfaceMesh dummy_surface_mesh(std::move(dummy_triangles),
                                         std::move(dummy_vertices));
  std::vector<int> dummy_surface_vertices = {0, 1, 2};
  std::vector<int> dummy_surface_tri_to_volume_tet = {0, 1, 2, 3};

  // Test copy-assignment operator.
  {
    DeformableGeometry copy(dummy_mesh, dummy_surface_mesh,
                            dummy_surface_vertices,
                            dummy_surface_tri_to_volume_tet);
    EXPECT_FALSE(copy.deformable_volume().mesh().Equal(
        original.deformable_volume().mesh()));
    EXPECT_FALSE(copy.deformable_surface().mesh().Equal(
        original.deformable_surface().mesh()));
    copy = original;

    // Test for uniqueness.
    EXPECT_NE(&original.deformable_volume(), &copy.deformable_volume());
    EXPECT_NE(&original.deformable_volume().mesh(),
              &copy.deformable_volume().mesh());
    EXPECT_NE(&copy.deformable_volume().bvh(),
              &original.deformable_volume().bvh());
    EXPECT_NE(&original.deformable_surface(), &copy.deformable_surface());

    EXPECT_TRUE(copy.deformable_volume().mesh().Equal(
        original.deformable_volume().mesh()));
    EXPECT_TRUE(copy.deformable_volume().bvh().Equal(
        original.deformable_volume().bvh()));
    EXPECT_TRUE(copy.deformable_surface().mesh().Equal(
        original.deformable_surface().mesh()));
    EXPECT_TRUE(copy.deformable_surface().bvh().Equal(
        original.deformable_surface().bvh()));

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
    EXPECT_NE(&original.deformable_volume(), &copy.deformable_volume());
    EXPECT_NE(&original.deformable_volume().mesh(),
              &copy.deformable_volume().mesh());
    EXPECT_NE(&copy.deformable_volume().bvh(),
              &original.deformable_volume().bvh());
    EXPECT_NE(&original.deformable_surface(), &copy.deformable_surface());

    EXPECT_TRUE(copy.deformable_volume().mesh().Equal(
        original.deformable_volume().mesh()));
    EXPECT_TRUE(copy.deformable_volume().bvh().Equal(
        original.deformable_volume().bvh()));
    EXPECT_TRUE(copy.deformable_surface().mesh().Equal(
        original.deformable_surface().mesh()));
    EXPECT_TRUE(copy.deformable_surface().bvh().Equal(
        original.deformable_surface().bvh()));

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
    const DeformableVolumeMeshWithBvh<double>* const mesh_ptr =
        &start.deformable_volume();
    const DeformableSurfaceMeshWithBvh<double>* const surface_mesh_ptr =
        &start.deformable_surface();

    // Test move constructor.
    DeformableGeometry move_constructed(std::move(start));
    EXPECT_EQ(&move_constructed.deformable_volume(), mesh_ptr);
    EXPECT_EQ(&move_constructed.deformable_surface(), surface_mesh_ptr);

    // Test move-assignment operator.
    DeformableGeometry move_assigned(dummy_mesh, dummy_surface_mesh,
                                     dummy_surface_vertices,
                                     dummy_surface_tri_to_volume_tet);
    move_assigned = std::move(move_constructed);
    EXPECT_EQ(&move_assigned.deformable_volume(), mesh_ptr);
    EXPECT_EQ(&move_assigned.deformable_surface(), surface_mesh_ptr);
  }
}

GTEST_TEST(DeformableGeometryTest, UpdateVertexPositions) {
  const Sphere sphere(1.0);
  VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(
      sphere, 0.5, TessellationStrategy::kDenseInteriorVertices);
  const int num_vertices = mesh.num_vertices();
  std::vector<int> surface_vertices;
  std::vector<int> surface_tri_to_volume_tet;
  TriangleSurfaceMesh<double> surface_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh, &surface_vertices, &surface_tri_to_volume_tet);
  const int num_surface_vertices = surface_mesh.num_vertices();
  DeformableGeometry deformable_geometry(
      std::move(mesh), std::move(surface_mesh), std::move(surface_vertices),
      std::move(surface_tri_to_volume_tet));
  const VectorXd q = VectorXd::LinSpaced(3 * num_vertices, 0.0, 1.0);
  const VectorXd q_surface =
      VectorXd::LinSpaced(3 * num_surface_vertices, 0.0, 1.0);
  deformable_geometry.UpdateVertexPositions(q, q_surface);
  const VolumeMesh<double>& deformed_mesh =
      deformable_geometry.deformable_volume().mesh();
  const TriangleSurfaceMesh<double>& deformed_surface_mesh =
      deformable_geometry.deformable_surface().mesh();
  for (int i = 0; i < num_vertices; ++i) {
    const Vector3d& q_MV = deformed_mesh.vertex(i);
    const Vector3d& expected_q_MV = q.segment<3>(3 * i);
    EXPECT_EQ(q_MV, expected_q_MV);
  }
  for (int i = 0; i < num_surface_vertices; ++i) {
    const Vector3d& q_MV = deformed_surface_mesh.vertex(i);
    const Vector3d& expected_q_MV = q_surface.segment<3>(3 * i);
    EXPECT_EQ(q_MV, expected_q_MV);
  }
}

GTEST_TEST(RigidGeometryTest, Pose) {
  const Sphere sphere(1.0);
  const double resolution_hint = 0.5;
  auto mesh = std::make_unique<VolumeMesh<double>>(MakeSphereVolumeMesh<double>(
      sphere, resolution_hint, TessellationStrategy::kSingleInteriorVertex));
  const double hydroelastic_modulus = 1234.5;
  auto mesh_field = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeSpherePressureField<double>(sphere, mesh.get(),
                                      hydroelastic_modulus));
  auto soft_hydro_mesh = make_unique<hydroelastic::SoftMesh>(
      std::move(mesh), std::move(mesh_field));
  RigidGeometry rigid_geometry(std::move(soft_hydro_mesh));
  const math::RigidTransform<double> X_WG(
      math::RollPitchYaw<double>(-1.57, 0, 3), Vector3d(-0.3, -0.55, 0.36));
  rigid_geometry.set_pose_in_world(X_WG);
  EXPECT_TRUE(X_WG.IsExactlyEqualTo(rigid_geometry.pose_in_world()));
}

GTEST_TEST(RigidGeometryTest, MakeMeshRepresentation) {
  const Sphere sphere(1.0);
  ProximityProperties props;
  const double resolution_hint = 0.5;
  AddCompliantHydroelasticProperties(resolution_hint, 1234.5, &props);
  EXPECT_TRUE(MakeMeshRepresentation(sphere, props).has_value());

  const MeshcatCone cone(1.0, 2.0, 3.0);
  EXPECT_FALSE(MakeMeshRepresentation(cone, props).has_value());

  props.AddProperty("hydroelastic", "slab_thickness", 0.1);
  const HalfSpace half_space;
  EXPECT_FALSE(MakeMeshRepresentation(half_space, props).has_value());
}

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
