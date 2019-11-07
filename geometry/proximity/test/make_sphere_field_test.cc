#include "drake/geometry/proximity/make_sphere_field.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MakeSphereFieldTest, MakeSpherePressureField) {
  // Use radius 2.0 to avoid identity scaling by 1.0.
  const Sphere sphere(2.0);
  // Use resolution_hint 0.25 to get a medium mesh with some boundary vertices
  // not exactly on the surface of the sphere due to numerical roundings. We do
  // not want to use the coarsest mesh (octahedron) since all vertices are
  // exactly on the coordinate axes.
  auto mesh = MakeSphereVolumeMesh<double>(sphere, 0.25);
  // Confirm that the mesh is not the coarsest one (octahedron).
  ASSERT_GT(mesh.num_vertices(), 7);
  ASSERT_GT(mesh.num_elements(), 8);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
    MakeSpherePressureField<double>(sphere, &mesh, kElasticModulus);

  // Check that all vertices have their pressure values within the range of
  // zero to kElasticModulus, and their minimum and maximum values are indeed
  // zero and kElasticModulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (VolumeVertexIndex v(0); v < mesh.num_vertices(); ++v) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_LE(pressure, kElasticModulus);
    EXPECT_GE(pressure, 0.0);
    if (pressure > max_pressure) {
      max_pressure = pressure;
    }
    if (pressure < min_pressure) {
      min_pressure = pressure;
    }
  }
  EXPECT_EQ(min_pressure, 0.0);
  EXPECT_EQ(max_pressure, kElasticModulus);

  // Check that all boundary vertices have zero pressure.
  std::vector<VolumeVertexIndex> boundary_vertex_indices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));
  for (const VolumeVertexIndex v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_EQ(pressure, 0.0);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
