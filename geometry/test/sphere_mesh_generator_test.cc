#include "drake/geometry/sphere_mesh_generator.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Computes the volume of a tetrahedron given the four vertices that define it.
// The convention is that the first three vertices a, b, c define a triangle
// with its right-handed normal pointing towards the outside of the tetrahedra.
// The fourth vertex, d, is on the negative side of the plane defined by a, b,
// c. With this convention, the computed volume will be positive, otherwise
// negative.
double CalcTetrahedronVolume(const Vector3<double>& a, const Vector3<double>& b,
                             const Vector3<double>& c,
                             const Vector3<double>& d) {
  return (a - d).dot((b - d).cross(c - d)) / 6.0;
}

// Computes the total volume of a TetrahedraMesh by summing up the contribution
// of each tetrahedra.
double CalcTetrahedraMeshVolume(const TetrahedraMesh<double>& mesh) {
  const std::vector<Vector3<double>>& vertices = mesh.vertices;
  const std::vector<Vector4<int>>& tetrahedra = mesh.tetrahedra;
  double volume = 0.0;
  for (const auto& t : tetrahedra) {
    volume += CalcTetrahedronVolume(vertices[t[0]], vertices[t[1]],
                                    vertices[t[2]], vertices[t[3]]);
  }
  return volume;
}

// This test verifies that the volume of the tessellated sphere converges to the
// exact value as the tessellation is refined.
GTEST_TEST(MakeSphereMesh, VolumeConvergence) {
  const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();

  auto mesh0 = SphereMeshGenerator<double>::MakeSphereMesh(0);
  const double volume0 = CalcTetrahedraMeshVolume(mesh0);
  const double sphere_volume = 4.0 / 3.0 * M_PI;
  // Initial error in the computation of the volume.
  double error0 = sphere_volume - volume0;

  // The volume of a pyramid with base of size l x w and height h is V = lwh/3.
  // For our level zero octahedron we have two pyramids with sizes
  // l = w = sqrt(2) and height = 1.0. Thus its volume is 4/3.
  const double expected_volume = 4.0 / 3.0;

  EXPECT_NEAR(volume0, expected_volume, kTolerance);

  for (int level = 1; level < 6; ++level) {
    auto mesh = SphereMeshGenerator<double>::MakeSphereMesh(level);

    // Verify correct size.
    const int num_tetrahedra = std::pow(8, level + 1);
    EXPECT_EQ(mesh.tetrahedra.size(), num_tetrahedra);

    // Verify that the volume monotonically converges towards the exact volume
    // of the unit radius sphere.
    const double volume = CalcTetrahedraMeshVolume(mesh);
    const double error = sphere_volume - volume;

    EXPECT_GT(error, 0.0);
    EXPECT_LT(error, error0);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
