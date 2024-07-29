#include "drake/geometry/proximity/pressure_field_invariants.h"

#include <algorithm>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

void VerifyInvariantsOfThePressureFieldWithMargin(
    const VolumeMeshFieldLinear<double, double>& field_no_magin,
    const VolumeMeshFieldLinear<double, double>& field_with_margin) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  // Assert that both fields are defined on the same mesh.
  ASSERT_EQ(&field_no_magin.mesh(), &field_with_margin.mesh());
  const VolumeMesh<double>& mesh = field_no_magin.mesh();

  const std::vector<int> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));

  double boundary_min_pressure = std::numeric_limits<double>::max();
  double boundary_max_pressure = std::numeric_limits<double>::lowest();
  for (int v : boundary_vertices) {
    const double p_with_margin = field_with_margin.EvaluateAtVertex(v);
    // Pressure with margin at the boundary must be negative.
    ASSERT_LT(p_with_margin, 0);
    boundary_min_pressure = std::min(boundary_min_pressure, p_with_margin);
    boundary_max_pressure = std::max(boundary_max_pressure, p_with_margin);
  }
  const double boundary_pressure =
      0.5 * (boundary_min_pressure + boundary_max_pressure);
  // Verify pressure at the boundary is a constant.
  const double tolerance = kEps * std::abs(boundary_pressure);
  EXPECT_NEAR(boundary_min_pressure, boundary_max_pressure, tolerance);

  // Build the set of interior vertices.
  std::vector<int> all_vertices(mesh.num_vertices());
  std::iota(all_vertices.begin(), all_vertices.end(), 0);
  std::vector<int> interior_vertices;
  std::set_difference(all_vertices.begin(), all_vertices.end(),
                      boundary_vertices.begin(), boundary_vertices.end(),
                      std::back_inserter(interior_vertices));

  // For interior vertices only, verify the linear relationship between the
  // pressure fields without and with margin.
  double min_slope = std::numeric_limits<double>::max();
  double max_slope = std::numeric_limits<double>::lowest();
  for (int v : interior_vertices) {
    const double p_no_margin = field_no_magin.EvaluateAtVertex(v);
    const double p_with_margin = field_with_margin.EvaluateAtVertex(v);
    const double slope = (p_with_margin - boundary_pressure) / p_no_margin;
    // The slope must larger than unity.
    ASSERT_GT(slope, 1);
    min_slope = std::min(min_slope, slope);
    max_slope = std::max(max_slope, slope);
  }
  // Verify slope is constant.
  EXPECT_NEAR(min_slope, max_slope, kEps);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
