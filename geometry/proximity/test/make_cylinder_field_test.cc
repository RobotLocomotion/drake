#include "drake/geometry/proximity/make_cylinder_field.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// TODO(DamrongGuoy): Consider sharing this function among all
//  make_`shape`_field_test.cc for box, sphere, ellipsoid, etc.
void CheckMinMaxBoundaryValue(
    const VolumeMeshFieldLinear<double, double>& pressure_field,
    const double elastic_modulus) {
  // Check that all vertices have their pressure values within the range of
  // zero to elastic_modulus, and their minimum and maximum values are indeed
  // zero and elastic_modulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (VolumeVertexIndex v(0); v < pressure_field.mesh().num_vertices(); ++v) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_LE(pressure, elastic_modulus);
    EXPECT_GE(pressure, 0.0);
    if (pressure > max_pressure) {
      max_pressure = pressure;
    }
    if (pressure < min_pressure) {
      min_pressure = pressure;
    }
  }
  EXPECT_EQ(min_pressure, 0.0);
  EXPECT_EQ(max_pressure, elastic_modulus);

  // Check that all boundary vertices have zero pressure.
  std::vector<VolumeVertexIndex> boundary_vertex_indices =
      CollectUniqueVertices(
          IdentifyBoundaryFaces(pressure_field.mesh().tetrahedra()));
  for (const VolumeVertexIndex& v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_EQ(pressure, 0.0);
  }

  // Check that the center (0,0,0) of the shape has the max_pressure.
  // This test assumes that the mesh has a vertex at the origin of its
  // canonical frame.
  VolumeVertexIndex center_vertex{0};
  for (VolumeVertexIndex v{0}; v < pressure_field.mesh().num_vertices(); ++v) {
    if (pressure_field.mesh().vertex(v).r_MV() == Vector3d::Zero()) {
      center_vertex = v;
      break;
    }
  }
  ASSERT_EQ(Vector3d::Zero(),
            pressure_field.mesh().vertex(center_vertex).r_MV());
  EXPECT_EQ(max_pressure, pressure_field.EvaluateAtVertex(center_vertex));
}

GTEST_TEST(MakeCylinderFieldTest, MakeCylinderPressureField) {
  // A cylinder with radius 5cm and length 30cm.
  const Cylinder cylinder(0.05, 0.30);
  // The resolution_hint 2cm should give a medium mesh with some boundary
  // vertices on the curved barrel surface that do not have exact coordinates.
  // In other words, we want the test to include these vertices, which are not
  // exactly on the cylinder surface due to numerical roundings. We do not
  // want to use the coarsest mesh (rectangular prism) for testing.
  const auto mesh = MakeCylinderVolumeMesh<double>(cylinder, 0.02);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeCylinderPressureField<double>(cylinder, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
