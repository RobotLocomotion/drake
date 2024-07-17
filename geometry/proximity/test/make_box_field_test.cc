#include "drake/geometry/proximity/make_box_field.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/sample_volume_field.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

using Eigen::Vector3d;

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MakeBoxFieldTest, MakeBoxPressureField) {
  // Box dimensions are set to avoid symmetry.
  const Box box(1.0, 2.0, 3.0);
  // The resolution 0.25 is fine enough to have vertices on the medial axis.
  auto mesh = MakeBoxVolumeMesh<double>(box, 0.25);
  // A reasonable positive value for testing the maximum pressure value.
  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeBoxPressureField<double>(box, &mesh, kElasticModulus);

  // Check that all vertices have their pressure values within the range of
  // zero to kElasticModulus, and their minimum and maximum values are indeed
  // zero and kElasticModulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (int v = 0; v < mesh.num_vertices(); ++v) {
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
  std::vector<int> boundary_vertex_indices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));
  for (int v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_EQ(pressure, 0.0);
  }
}

GTEST_TEST(MakeBoxFieldTest, MakeBoxPressureFieldWithMargin) {
  const double kMarginValue = 1.5e-3;
  const double kEps = std::numeric_limits<double>::epsilon();
  // Box dimensions are set to avoid symmetry.
  const Box box(1.0, 2.0, 3.0);
  const Box inflated_box(box.size() + 2.0 * Vector3d::Constant(kMarginValue));
  // Make meshes for the original and inflated geometries such that both have
  // the same connectivity.
  auto mesh = MakeBoxVolumeMeshWithMa<double>(box);
  auto inflated_mesh = MakeBoxVolumeMeshWithMa<double>(inflated_box);
  // A reasonable positive value for testing the maximum pressure value.
  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeBoxPressureField<double>(inflated_box, &inflated_mesh,
                                   kElasticModulus, kMarginValue);

  // Expected min pressure and gradient at the boundary, based on the original
  // geometry.
  const double min_half_size = box.size().minCoeff() / 2.0;
  const double expected_pressure_gradient = kElasticModulus / min_half_size;
  const double expected_min_pressure =
      -expected_pressure_gradient * kMarginValue;
  const double kMinPressureSlop = std::abs(expected_min_pressure) * kEps;

  // Check that all vertices have their pressure values within the range of
  // expected_min_pressure to kElasticModulus.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (int v = 0; v < inflated_mesh.num_vertices(); ++v) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_LE(pressure, kElasticModulus);
    EXPECT_GE(pressure, expected_min_pressure - kMinPressureSlop);
    max_pressure = std::max(pressure, max_pressure);
    min_pressure = std::min(pressure, min_pressure);
  }
  EXPECT_EQ(max_pressure, kElasticModulus);
  EXPECT_NEAR(min_pressure, expected_min_pressure, kMinPressureSlop);

  // Check that all boundary vertices have expected_min_pressure.
  std::vector<int> boundary_vertex_indices =
      CollectUniqueVertices(IdentifyBoundaryFaces(inflated_mesh.tetrahedra()));
  for (int v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_NEAR(pressure, expected_min_pressure, kMinPressureSlop);
  }

  // Verify that the zero level set on the inflated mesh represents the original
  // geometry.
  ASSERT_EQ(mesh.num_vertices(), inflated_mesh.num_vertices());
  for (int v : boundary_vertex_indices) {
    const Vector3d& p = mesh.vertex(v);
    const double pressure = SampleVolumeFieldOrThrow(pressure_field, p);
    EXPECT_NEAR(pressure / kElasticModulus, 0.0, kEps);
  }
}

GTEST_TEST(MakeBoxFieldTest, MakeBoxPressureFieldInMeshWithMedialAxis) {
  // Box dimensions are set to avoid symmetry.
  const Box box(1.0, 2.0, 3.0);
  const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);

  // A reasonable positive value for testing the maximum pressure value.
  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeBoxPressureField<double>(box, &mesh, kElasticModulus);

  // Check that all vertices have their pressure values within the range of
  // zero to kElasticModulus, and their minimum and maximum values are indeed
  // zero and kElasticModulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (int v = 0; v < mesh.num_vertices(); ++v) {
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
  std::vector<int> boundary_vertex_indices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));
  for (int v : boundary_vertex_indices) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_EQ(pressure, 0.0);
  }

  // Check that there is no constant-pressure tetrahedron.
  const double kPressureTolerance =
      kElasticModulus * std::numeric_limits<double>::epsilon();
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const double pressure_v0 =
        pressure_field.EvaluateAtVertex(mesh.element(e).vertex(0));
    bool same_pressure = true;
    for (int i = 1; i < mesh.kVertexPerElement && same_pressure; ++i) {
      const double pressure_vi =
          pressure_field.EvaluateAtVertex(mesh.element(e).vertex(i));
      same_pressure = kPressureTolerance >= std::abs(pressure_v0 - pressure_vi);
    }
    EXPECT_FALSE(same_pressure);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
