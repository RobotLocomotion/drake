#include "drake/geometry/proximity/make_convex_field.h"

#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/make_convex_mesh.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// TODO(DamrongGuoy): Consider sharing this function among all
// make_`shape`_field_test.cc for box, sphere, ellipsoid, etc.

// Checks that the pressure values evaluated at each vertex of the underlying
// mesh of `pressure_field` are within the range [0, hydroelastic_modulus]. Also
// checks that the pressure values evaluated at boundary vertices are 0.
void CheckMinMaxBoundaryValue(
    const VolumeMeshFieldLinear<double, double>& pressure_field,
    const double hydroelastic_modulus) {
  // Check that all vertices have their pressure values within the range of
  // zero to hydroelastic_modulus, and their minimum and maximum values are
  // indeed zero and hydroelastic_modulus respectively.
  double max_pressure = std::numeric_limits<double>::lowest();
  double min_pressure = std::numeric_limits<double>::max();
  for (int v = 0; v < pressure_field.mesh().num_vertices(); ++v) {
    const double pressure = pressure_field.EvaluateAtVertex(v);
    ASSERT_LE(pressure, hydroelastic_modulus);
    ASSERT_GE(pressure, 0.0);
    if (pressure > max_pressure) {
      max_pressure = pressure;
    }
    if (pressure < min_pressure) {
      min_pressure = pressure;
    }
  }
  EXPECT_EQ(min_pressure, 0.0);
  EXPECT_EQ(max_pressure, hydroelastic_modulus);

  // TODO(joemasterjohn): Rather than searching for boundary vertices, use
  // type traits to access vertices with a particular property using a priori
  // knowledge of the mesh structure. This goes for medial axis vertices also.

  // Check that all boundary vertices have zero pressure.
  std::vector<int> boundary_vertex_indices = CollectUniqueVertices(
      IdentifyBoundaryFaces(pressure_field.mesh().tetrahedra()));
  for (int v : boundary_vertex_indices) {
    ASSERT_EQ(pressure_field.EvaluateAtVertex(v), 0.0);
  }
}

// Instantiate a parameterized test suite. A single parameter for the mesh
// filename is provided.
class MakeConvexFieldTest : public testing::TestWithParam<std::string> {};

INSTANTIATE_TEST_SUITE_P(ConvexField, MakeConvexFieldTest,
                         testing::Values("drake/geometry/test/quad_cube.obj",
                                         "drake/geometry/test/convex.obj"));

TEST_P(MakeConvexFieldTest, CheckMinMaxBoundaryValue) {
  std::string filename = GetParam();

  std::string mesh_file = FindResourceOrThrow(filename);

  VolumeMesh<double> volume_mesh =
      MakeConvexVolumeMesh<double>(Convex(mesh_file));
  const double kElasticModulus = 1.0e5;
  VolumeMeshFieldLinear<double, double> pressure_field =
      MakeConvexPressureField<double>(&volume_mesh, kElasticModulus);

  CheckMinMaxBoundaryValue(pressure_field, kElasticModulus);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
