#include "drake/geometry/proximity/make_unit_sphere_mesh.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// TODO(amcastro-tri): The unit tests below were designed with the idea in mind
// that if a typical bug is present (for instance wrong tetrahedron sign
// convention, missing tetrahedra or faulty vertices computation) they would
// fail. However, from a purist point of view on unit testing, a variety of
// tests are missing. To mention a few:
//   1. Level 0 tessellation should have all of its properties confirmed:
//      tetrahedra sign convention, positive volume, sphere coverage.
//   2. Is level 0 correct in that there are no overlappong tetrahedra?
//   3. The refinement process preserves the tetrahedron sign convention.
//   4. Confirm that the k-level refinement of a tet spans the same volume of
//      the k-1-level (if there are no boundary vertices).
//
// N.B. All of these were confirmed during the development however they did not
// make it into a clean unit test.

// Computes the total volume of a VolumeMesh by summing up the contribution
// of each tetrahedron.
double CalcTetrahedronMeshVolume(const VolumeMesh<double>& mesh) {
  double volume = 0.0;
  for (int e = 0; e < mesh.num_elements(); ++e) {
    volume += mesh.CalcTetrahedronVolume(VolumeElementIndex(e));
  }
  return volume;
}

// This test verifies that the volume of the tessellated sphere converges to the
// exact value as the tessellation is refined.
GTEST_TEST(MakeSphereMesh, VolumeConvergence) {
  const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();

  auto mesh0 = MakeUnitSphereMesh<double>(0);
  const double volume0 = CalcTetrahedronMeshVolume(mesh0);
  const double sphere_volume = 4.0 / 3.0 * M_PI;
  // Initial error in the computation of the volume.
  double prev_error = sphere_volume - volume0;

  // The volume of a pyramid with base of size l x w and height h is V = lwh/3.
  // For our level zero octahedron we have two pyramids with sizes
  // l = w = sqrt(2) and height = 1.0. Thus its volume is 4/3.
  const double expected_volume = 4.0 / 3.0;

  EXPECT_NEAR(volume0, expected_volume, kTolerance);

  for (int level = 1; level < 6; ++level) {
    auto mesh = MakeUnitSphereMesh<double>(level);

    // Verify correct size.
    const size_t num_tetrahedra = std::pow(8, level + 1);
    EXPECT_EQ(mesh.tetrahedra().size(), num_tetrahedra);

    // Verify that the volume monotonically converges towards the exact volume
    // of the unit radius sphere.
    const double volume = CalcTetrahedronMeshVolume(mesh);
    const double error = sphere_volume - volume;

    EXPECT_GT(error, 0.0);
    EXPECT_LT(error, prev_error);

    // Always compare against last computed error to show monotonic convergence.
    prev_error = error;
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
