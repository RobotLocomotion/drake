#include "drake/geometry/proximity/test/pressure_field_invariants.h"

#include <algorithm>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

void VerifyInvariantsOfThePressureFieldWithMargin(
    const VolumeMeshFieldLinear<double, double>& field_no_margin,
    const VolumeMeshFieldLinear<double, double>& field_with_margin,
    double margin, double elastic_foundation_depth, double hydroelastic_modulus,
    double relative_tolerance) {
  // Assert that both fields are defined on the same mesh.
  DRAKE_DEMAND(&field_no_margin.mesh() == &field_with_margin.mesh());
  const VolumeMesh<double>& mesh = field_no_margin.mesh();

  const std::vector<int> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));

  // Verify that pressure at the boundary is constant and equal to p̃₀ =
  // −δ/(H−δ)⋅E .
  const double expected_boundary_pressure =
      -margin / (elastic_foundation_depth - margin) * hydroelastic_modulus;
  const double tolerance =
      relative_tolerance * std::abs(expected_boundary_pressure);
  for (int v : boundary_vertices) {
    const double p_with_margin = field_with_margin.EvaluateAtVertex(v);
    // Pressure with margin at the boundary must be negative.
    ASSERT_LT(p_with_margin, 0);
    ASSERT_NEAR(p_with_margin, expected_boundary_pressure, tolerance);
  }

  // Build the set of interior vertices.
  std::vector<int> all_vertices(mesh.num_vertices());
  std::iota(all_vertices.begin(), all_vertices.end(), 0);
  std::vector<int> interior_vertices;
  std::set_difference(all_vertices.begin(), all_vertices.end(),
                      boundary_vertices.begin(), boundary_vertices.end(),
                      std::back_inserter(interior_vertices));

  // For interior vertices only, verify the linear relationship between the
  // pressure fields without and with margin.
  const double expected_slope =
      elastic_foundation_depth / (elastic_foundation_depth - margin);
  for (int v : interior_vertices) {
    const double p_no_margin = field_no_margin.EvaluateAtVertex(v);
    const double p_with_margin = field_with_margin.EvaluateAtVertex(v);
    const double slope =
        (p_with_margin - expected_boundary_pressure) / p_no_margin;
    ASSERT_NEAR(slope, expected_slope, relative_tolerance);
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
