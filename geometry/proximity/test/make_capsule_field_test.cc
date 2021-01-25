#include "drake/geometry/proximity/make_capsule_field.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/make_capsule_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

void CheckMinMaxBoundaryValue(
    const VolumeMeshFieldLinear<double, double>& pressure_field,
    const double elastic_modulus) {
  // We pick the relative error 1e-14 of the elastic modulus empirically.
  const double tolerance = 1e-14 * elastic_modulus;
  // Check that all vertices have their pressure values within the range of
  // zero to elastic_modulus, and their minimum and maximum values are indeed
  // zero and elastic_modulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (VolumeVertexIndex v(0); v < pressure_field.mesh().num_vertices(); ++v) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    ASSERT_LE(pressure, elastic_modulus + tolerance);
    ASSERT_GE(pressure, 0.0);
    if (pressure > max_pressure) {
      max_pressure = pressure;
    }
    if (pressure < min_pressure) {
      min_pressure = pressure;
    }
  }
  EXPECT_EQ(min_pressure, 0.0);
  EXPECT_NEAR(max_pressure, elastic_modulus, tolerance);

  // Check that all boundary vertices have zero pressure.
  std::vector<VolumeVertexIndex> boundary_vertex_indices =
      CollectUniqueVertices(
          IdentifyBoundaryFaces(pressure_field.mesh().tetrahedra()));
  for (const VolumeVertexIndex& v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    ASSERT_EQ(pressure, 0.0);
  }

  // This test only applies to a mesh that has a vertex at the center of
  // the geometric shape, where we check that the center vertex has the
  // max_pressure.
  VolumeVertexIndex center_vertex{0};
  bool has_center_vertex = false;
  for (VolumeVertexIndex v{0}; v < pressure_field.mesh().num_vertices(); ++v) {
    if (pressure_field.mesh().vertex(v).r_MV() == Vector3d::Zero()) {
      center_vertex = v;
      has_center_vertex = true;
      break;
    }
  }
  if (has_center_vertex) {
    ASSERT_EQ(Vector3d::Zero(),
              pressure_field.mesh().vertex(center_vertex).r_MV());
    EXPECT_NEAR(max_pressure, pressure_field.EvaluateAtVertex(center_vertex),
                tolerance);
  }
}

// Test the coarsest possible mesh.
GTEST_TEST(MakeCapsuleFieldTest, MakeCapsulePressureFieldCoarsest) {
  const double radius = 0.5;
  const double length = 2.0;
  // Number of vertices per circluar rim of the capsule.
  const int n = 3;
  const double resolution_hint = 2.0 * M_PI * radius / n;

  const Capsule capsule(radius, length);
  const auto mesh = MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeCapsulePressureField<double>(capsule, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

// Test a medium resolution mesh.
GTEST_TEST(MakeCapsuleFieldTest, MakeCapsulePressureFieldMedium) {
  const double radius = 0.5;
  const double length = 2.0;
  // Number of vertices per circluar rim of the capsule.
  const int n = 80;
  const double resolution_hint = 2.0 * M_PI * radius / n;

  const Capsule capsule(radius, length);
  const auto mesh = MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeCapsulePressureField<double>(capsule, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

// Test the finest possible mesh.
GTEST_TEST(MakeCapsuleFieldTest, MakeCapsulePressureFieldFinest) {
  const double radius = 0.5;
  const double length = 2.0;
  // Number of vertices per circluar rim of the capsule.
  // See MakeCapsuleVolumeMesh() for the explanation of the maximum value for n.
  const int n = 706;
  const double resolution_hint = 2.0 * M_PI * radius / n;

  const Capsule capsule(radius, length);
  const auto mesh = MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeCapsulePressureField<double>(capsule, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
