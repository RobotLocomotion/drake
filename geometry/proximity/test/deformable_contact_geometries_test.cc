#include "drake/geometry/proximity/deformable_contact_geometries.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;

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
                      internal::HydroelasticType::kSoft);
  } else {
    DRAKE_UNREACHABLE();
  }
  return props;
}

// TODO(xuchenhan-tri): The following test if is far from complete. It needs to
// cover other shapes than spheres and the test on mesh generation needs to be
// strengthened as well.

GTEST_TEST(DeformableContactGeometriesTest, MakeDeformableGeometry) {
  // Make sure we get the coarsest sphere volume mesh (an octahedron).
  const ProximityProperties props =
      MakeProximityPropsWithRezHint(GeometryType::kDeformable, 100);
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

  // We leverage our knowledge on sphere mesh generation -- the targeted edge
  // length is set to the coarsest possible, so the generated mesh is an
  // octahedron. So we expect 7 vertices: 6 on the surface of the sphere and 1
  // at the center.
  EXPECT_EQ(num_vertices, 7);
  const VolumeMeshFieldLinear<double, double>& sdf =
      deformable_sphere.signed_distance();
  EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(0), 1.0);
  for (int i = 1; i < mesh.num_vertices(); ++i) {
    EXPECT_DOUBLE_EQ(sdf.EvaluateAtVertex(i), 0.0);
  }
}

GTEST_TEST(DeformableContactGeometriesTest, MakeRigidGeometry) {
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

}  // namespace
}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
