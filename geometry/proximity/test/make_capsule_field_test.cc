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

// TODO(DamrongGuoy): Consider sharing this function among all
// make_`shape`_field_test.cc for box, sphere, ellipsoid, etc.

// Checks that the pressure values evaluated at each vertex of the underlying
// mesh of `pressure_field` are within the range [0, elastic_modulus]. Also
// checks that the pressure values evaluate at boundary vertices are 0.
void CheckMinMaxBoundaryValue(
    const VolumeMeshFieldLinear<double, double>& pressure_field,
    double elastic_modulus_in) {
  const double elastic_modulus = elastic_modulus_in;
  // Check that all vertices have their pressure values within the range of
  // zero to elastic_modulus, and their minimum and maximum values are indeed
  // zero and elastic_modulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (VolumeVertexIndex v(0); v < pressure_field.mesh().num_vertices(); ++v) {
    const double pressure = pressure_field.EvaluateAtVertex(v);
    ASSERT_LE(pressure, elastic_modulus);
    ASSERT_GE(pressure, 0.0);
    if (pressure > max_pressure) {
      max_pressure = pressure;
    }
    if (pressure < min_pressure) {
      min_pressure = pressure;
    }
  }
  EXPECT_EQ(min_pressure, 0.0);
  EXPECT_EQ(max_pressure, elastic_modulus);

  // TODO(joemasterjohn): Rather than searching for boundary vertices, use
  // type traits to access vertices with a particular property using a priori
  // knowledge of the mesh structure. This goes for medial axis vertices also.

  // Check that all boundary vertices have zero pressure.
  std::vector<VolumeVertexIndex> boundary_vertex_indices =
      CollectUniqueVertices(
          IdentifyBoundaryFaces(pressure_field.mesh().tetrahedra()));
  for (const VolumeVertexIndex& v : boundary_vertex_indices) {
    ASSERT_EQ(pressure_field.EvaluateAtVertex(v), 0.0);
  }
}

// Instantiate a parameterized test suite. A single parameter "number of
// vertices per circular rim" is provided.
class MakeCapsuleFieldTest : public testing::TestWithParam<int> {};

INSTANTIATE_TEST_SUITE_P(CapsuleField, MakeCapsuleFieldTest, testing::Values(
    3,  // Coarsest possible mesh.
    20  // Fine mesh.
));

TEST_P(MakeCapsuleFieldTest, CheckMinMaxBoundaryValue) {
  const double radius = 0.5;
  const double length = 2.0;
  // Number of vertices per circluar rim of the capsule.
  const int n = GetParam();
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
