#include "drake/geometry/proximity/make_ellipsoid_field.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/sample_volume_field.h"

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

GTEST_TEST(MakeEllipsoidFieldTest, MakeEllipsoidPressureField) {
  // For an ellipsoid with bounding box 10cm x 16cm x 6cm, its semi-axes are
  // 5cm, 8cm, and 3cm long.
  const Ellipsoid ellipsoid(0.05, 0.08, 0.03);
  // Use resolution_hint 4cm to get a medium mesh with some boundary vertices
  // not exactly on the surface of the ellipsoid due to numerical roundings.
  // We do not want to use the coarsest mesh (octahedron) since all vertices
  // are exactly on the coordinate axes.
  auto mesh = MakeEllipsoidVolumeMesh<double>(
      ellipsoid, 0.04, TessellationStrategy::kDenseInteriorVertices);
  // Confirm that the mesh is not the coarsest one (octahedron).
  ASSERT_GT(mesh.num_vertices(), 7);
  ASSERT_GT(mesh.num_elements(), 8);

  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeEllipsoidPressureField<double>(ellipsoid, &mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

GTEST_TEST(MakeEllipsoidFieldTest, MakeEllipsoidPressureFieldWithMargin) {
  const double kMargin = 1.3e-2;
  // For an ellipsoid with bounding box 10cm x 16cm x 6cm, its semi-axes are
  // 5cm, 8cm, and 3cm long.
  const Ellipsoid ellipsoid(0.05, 0.08, 0.03);
  const Ellipsoid inflated_ellipsoid(ellipsoid.a() + kMargin,
                                     ellipsoid.b() + kMargin,
                                     ellipsoid.c() + kMargin);
  // Use resolution_hint 4cm to get a medium mesh with some boundary vertices
  // not exactly on the surface of the ellipsoid due to numerical roundings.
  // We do not want to use the coarsest mesh (octahedron) since all vertices
  // are exactly on the coordinate axes.
  auto mesh = MakeEllipsoidVolumeMesh<double>(
      ellipsoid, 0.04, TessellationStrategy::kDenseInteriorVertices);
  auto inflated_mesh = MakeEllipsoidVolumeMesh<double>(
      inflated_ellipsoid, 0.04, TessellationStrategy::kDenseInteriorVertices);

  // Confirm that the mesh is not the coarsest one (octahedron).
  ASSERT_GT(mesh.num_vertices(), 7);
  ASSERT_GT(mesh.num_elements(), 8);

  // For such small margin we expect the connectivity of these two meshes to be
  // the same.
  ASSERT_EQ(mesh.num_vertices(), inflated_mesh.num_vertices());
  ASSERT_EQ(mesh.tetrahedra(), inflated_mesh.tetrahedra());

  const double kElasticModulus = 1.0e5;
  const VolumeMeshFieldLinear<double, double> pressure_field =
      MakeEllipsoidPressureField<double>(inflated_ellipsoid, &inflated_mesh,
                                         kElasticModulus, kMargin);

  const double foundation_length =
      std::min(ellipsoid.a(), std::min(ellipsoid.b(), ellipsoid.c()));
  const double expected_min_pressure =
      -kElasticModulus * kMargin / foundation_length;

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus,
                           expected_min_pressure);

  // Verify pressure is zero when evaluated on the surface of the original
  // ellipsoid.
  std::vector<int> boundary_vertex_indices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));
  double rms_error = 0;
  for (int v : boundary_vertex_indices) {
    const Vector3d& p = mesh.vertex(v);
    const double pressure = SampleVolumeFieldOrThrow(pressure_field, p);
    rms_error += pressure * pressure;
    // EXPECT_NEAR(pressure / kElasticModulus, 0.0, kEps);
  }
  rms_error = std::sqrt(rms_error / ssize(boundary_vertex_indices));
  rms_error /= kElasticModulus;  // normalize.

  // The inflated mesh is not conformal to the original mesh. Therefore we
  // interpolated values on the surface by SampleVolumeFieldOrThrow() will not
  // be exactly zero due to (first order) interpolation errors. We have verified
  // this errors does indeed go to zero as we refine the mesh. For the purposes
  // of this test however, we only check the rms error is below a value obtained
  // from these previous runs.
  EXPECT_LT(rms_error, 0.03);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
