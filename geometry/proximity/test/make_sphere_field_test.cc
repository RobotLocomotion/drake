#include "drake/geometry/proximity/make_sphere_field.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// TODO(DamrongGuoy): Consider sharing this function among all
//  make_`shape`_field_test.cc for box, sphere, ellipsoid, etc.
void CheckMinMaxBoundaryValue(
    const VolumeMeshFieldLinear<double, double>& pressure_field,
    const double hydroelastic_modulus) {
  // Check that all vertices have their pressure values within the range of
  // zero to hydroelastic_modulus, and their minimum and maximum values are
  // indeed zero and hydroelastic_modulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (int v = 0; v < pressure_field.mesh().num_vertices(); ++v) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_LE(pressure, hydroelastic_modulus);
    EXPECT_GE(pressure, 0.0);
    if (pressure > max_pressure) {
      max_pressure = pressure;
    }
    if (pressure < min_pressure) {
      min_pressure = pressure;
    }
  }
  EXPECT_EQ(min_pressure, 0.0);
  EXPECT_EQ(max_pressure, hydroelastic_modulus);

  // Check that all boundary vertices have zero pressure.
  std::vector<int> boundary_vertex_indices =
      CollectUniqueVertices(
          IdentifyBoundaryFaces(pressure_field.mesh().tetrahedra()));
  for (int v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_EQ(pressure, 0.0);
  }

  // Check that the center (0,0,0) of the shape has the max_pressure.
  // This test assumes that the mesh has a vertex at the origin of its
  // canonical frame.
  int center_vertex = 0;
  for (int v = 0; v < pressure_field.mesh().num_vertices(); ++v) {
    if (pressure_field.mesh().vertex(v) == Vector3d::Zero()) {
      center_vertex = v;
      break;
    }
  }
  ASSERT_EQ(Vector3d::Zero(), pressure_field.mesh().vertex(center_vertex));
  EXPECT_EQ(max_pressure, pressure_field.EvaluateAtVertex(center_vertex));
}

GTEST_TEST(MakeSphereFieldTest, MakeSpherePressureField) {
  // Use radius 2.0 to avoid identity scaling by 1.0.
  const Sphere sphere(2.0);
  // Use resolution_hint 0.25 to get a medium mesh with some boundary vertices
  // not exactly on the surface of the sphere due to numerical roundings. We do
  // not want to use the coarsest mesh (octahedron) since all vertices are
  // exactly on the coordinate axes.
  auto mesh = MakeSphereVolumeMesh<double>(
      sphere, 0.25, TessellationStrategy::kDenseInteriorVertices);
  // Confirm that the mesh is not the coarsest one (octahedron).
  ASSERT_GT(mesh.num_vertices(), 7);
  ASSERT_GT(mesh.num_elements(), 8);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
    MakeSpherePressureField<double>(sphere, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
