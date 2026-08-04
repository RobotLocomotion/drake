#include "drake/geometry/proximity/make_cylinder_field.h"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/test/pressure_field_invariants.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// TODO(DamrongGuoy): Consider sharing this function among all
//  make_`shape`_field_test.cc for box, sphere, ellipsoid, etc.
void CheckMinMaxBoundaryValue(
    const VolumeMeshFieldLinear<double, double>& pressure_field,
    double hydroelastic_modulus, double expected_min_pressure = 0) {
  // We pick the relative error 1e-14 of the elastic modulus empirically.
  const double tolerance = 1e-14 * hydroelastic_modulus;
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  // N.B. The slop is zero, thus exact comparison, when the expected minimum
  // pressure is zero.
  const double min_pressure_slop = kEps * std::abs(expected_min_pressure);
  // Check that all vertices have their pressure values within the range of
  // zero to hydroelastic_modulus, and their minimum and maximum values are
  // indeed zero and hydroelastic_modulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (int v = 0; v < pressure_field.mesh().num_vertices(); ++v) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    ASSERT_LE(pressure, hydroelastic_modulus + tolerance);
    ASSERT_GE(pressure, expected_min_pressure - min_pressure_slop);
    if (pressure > max_pressure) {
      max_pressure = pressure;
    }
    if (pressure < min_pressure) {
      min_pressure = pressure;
    }
  }
  EXPECT_NEAR(min_pressure, expected_min_pressure, min_pressure_slop);
  EXPECT_NEAR(max_pressure, hydroelastic_modulus, tolerance);

  // Check that all boundary vertices have zero pressure.
  std::vector<int> boundary_vertex_indices = CollectUniqueVertices(
      IdentifyBoundaryFaces(pressure_field.mesh().tetrahedra()));
  for (int v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_NEAR(pressure, expected_min_pressure, min_pressure_slop);
  }

  // This test only applies to a mesh that has a vertex at the center of
  // the geometric shape, where we check that the center vertex has the
  // max_pressure.
  int center_vertex = 0;
  bool has_center_vertex = false;
  for (int v = 0; v < pressure_field.mesh().num_vertices(); ++v) {
    if (pressure_field.mesh().vertex(v) == Vector3d::Zero()) {
      center_vertex = v;
      has_center_vertex = true;
      break;
    }
  }
  if (has_center_vertex) {
    ASSERT_EQ(Vector3d::Zero(), pressure_field.mesh().vertex(center_vertex));
    EXPECT_NEAR(max_pressure, pressure_field.EvaluateAtVertex(center_vertex),
                tolerance);
  }
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

GTEST_TEST(MakeCylinderFieldTest, MakeCylinderPressureFieldWithMaLong) {
  const Cylinder cylinder(1., 3.);
  const VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, 0.25);
  const double kElasticModulus = 1.0e5;
  const VolumeMeshFieldLinear<double, double> pressure_field =
      MakeCylinderPressureField<double>(cylinder, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

GTEST_TEST(MakeCylinderFieldTest, MakeCylinderPressureFieldWithMaMedium) {
  const Cylinder cylinder(1., 2.);
  const VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, 0.25);
  const double kElasticModulus = 1.0e5;
  const VolumeMeshFieldLinear<double, double> pressure_field =
      MakeCylinderPressureField<double>(cylinder, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

GTEST_TEST(MakeCylinderFieldTest, MakeCylinderPressureFieldWithMaShort) {
  const Cylinder cylinder(1., 1.);
  const VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, 0.25);
  const double kElasticModulus = 1.0e5;
  const VolumeMeshFieldLinear<double, double> pressure_field =
      MakeCylinderPressureField<double>(cylinder, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

GTEST_TEST(MakeCylinderFieldTest, WithMargin) {
  const double kElasticModulus = 1.0e5;
  const double kMargin = 0.12;
  const Cylinder cylinder(1., 3.);
  const double elastic_foundation_depth = 1.0;  // min half-size.
  const VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, 0.25);
  const VolumeMeshFieldLinear<double, double> field_no_margin =
      MakeCylinderPressureField<double>(cylinder, &mesh, kElasticModulus);
  const VolumeMeshFieldLinear<double, double> field_with_margin =
      MakeCylinderPressureField<double>(cylinder, &mesh, kElasticModulus,
                                        kMargin);
  VerifyInvariantsOfThePressureFieldWithMargin(
      field_no_margin, field_with_margin, kMargin, elastic_foundation_depth,
      kElasticModulus);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
