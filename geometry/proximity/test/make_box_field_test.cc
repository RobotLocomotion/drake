#include "drake/geometry/proximity/make_box_field.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MakeBoxFieldTest, MakeBoxPressureField) {
  const Box box(1.0, 2.0, 3.0);
  auto mesh = MakeBoxVolumeMesh<double>(box, 0.25);
  const double kElasticModulus = 1.0e5;
  auto linear_pressure = [&kElasticModulus](double extent) {
    return kElasticModulus * extent;
  };
  auto pressure_field =
      MakeBoxPressureField<double>(box, mesh, linear_pressure);

  // Check that all vertices have their pressure values between zero and
  // kElasticModulus.
  for (VolumeVertexIndex v(0); v < mesh.num_vertices(); ++v) {
    double pressure = pressure_field.EvaluateAtVertex(v);
    EXPECT_LE(pressure, kElasticModulus);
    EXPECT_GE(pressure, 0.0);
  }

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
