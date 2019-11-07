#include "drake/geometry/proximity/make_ellipsoid_field.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_ellipsoid_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// TODO(DamrongGuoy): Consider sharing this function among all
//  make_`shape`_field_test.cc for box, sphere, ellipsoid, etc.
void CheckMinMaxBoundaryValue(
    const VolumeMeshFieldLinear<double, double>& pressure_field,
    const double kElasticModulus) {
  // Check that all vertices have their pressure values within the range of
  // zero to kElasticModulus, and their minimum and maximum values are indeed
  // zero and kElasticModulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (VolumeVertexIndex v(0); v < pressure_field.mesh().num_vertices(); ++v) {
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
      CollectUniqueVertices(
          IdentifyBoundaryFaces(pressure_field.mesh().tetrahedra()));
  for (const VolumeVertexIndex v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_EQ(pressure, 0.0);
  }
}

GTEST_TEST(MakeEllipsoidFieldTest, MakeEllipsoidPressureField) {
  // For an ellipsoid with bounding box 10cm x 16cm x 6cm, its semi-axes are
  // 5cm, 8cm, and 3cm long.
  const Ellipsoid ellipsoid(0.05, 0.08, 0.03);
  // Use resolution_hint 4cm to get a medium mesh with some boundary vertices
  // not exactly on the surface of the ellipsoid due to numerical roundings.
  // We do not want to use the coarsest mesh (octahedron) since all vertices
  // are exactly on the coordinate axes.
  auto mesh = MakeEllipsoidVolumeMesh<double>(ellipsoid, 0.04);
  // Confirm that the mesh is not the coarsest one (octahedron).
  ASSERT_GT(mesh.num_vertices(), 7);
  ASSERT_GT(mesh.num_elements(), 8);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeEllipsoidPressureField<double>(ellipsoid, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
