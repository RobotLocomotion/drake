#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <algorithm>
#include <fstream>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Computes the volume of a tetrahedron given the four vertices that define it.
// The convention is that the first three vertices a, b, c define a triangle
// with its right-handed normal pointing towards the inside of the tetrahedra.
// The fourth vertex, d, is on the positive side of the plane defined by a, b,
// c. With this convention, the computed volume will be positive, otherwise
// negative.
double CalcTetrahedronVolume(const Vector3<double>& a, const Vector3<double>& b,
                             const Vector3<double>& c,
                             const Vector3<double>& d) {
  return (d - a).dot((b - a).cross(c - a)) / 6.0;
}

// Computes the total volume of a VolumeMesh by summing up the contribution
// of each tetrahedron.
double CalcTetrahedronMeshVolume(const VolumeMesh<double>& mesh) {
  const std::vector<VolumeVertex<double>> &vertices = mesh.vertices();
  const std::vector<VolumeElement> &tetrahedra = mesh.tetrahedra();
  double volume = 0.0;
  for (const auto &t : tetrahedra) {
    volume += CalcTetrahedronVolume(
        vertices[t.vertex(0)].r_MV(), vertices[t.vertex(1)].r_MV(),
        vertices[t.vertex(2)].r_MV(), vertices[t.vertex(3)].r_MV());
  }
  return volume;
}

// This test verifies that the volume of the tessellated
// cylinder converges to the exact value as the tessellation is refined.
GTEST_TEST(MakeCylinderMesh, VolumeConvergence) {
  const double kTolerance = 10.0 * std::numeric_limits<double>::epsilon();

  const double height = 2;
  const double radius = 1;
  const int refinement_level = 0;
  const int smoothing_iterations = 0;

  auto mesh0 = MakeCylinderMesh<double>(height, radius,
      refinement_level, smoothing_iterations);

  const double volume0 = CalcTetrahedronMeshVolume(mesh0);
  const double cylinder_volume = height * radius * radius * M_PI;

  // Initial error in the computation of the volume.
  double prev_error = cylinder_volume - volume0;

  // The volume of a rectangular prism with base
  // of size l x w and height h is V = lwh.
  // For our level zero cylinder, we have a prism of height 2 and
  // l = w = sqrt(2) and height = 2.0. Thus its volume is 4.
  const double expected_volume = 4.0;

  EXPECT_NEAR(volume0, expected_volume, kTolerance);

  for (int level = 1; level < 6; ++level) {
    auto mesh = MakeCylinderMesh<double>(height, radius,
        level, smoothing_iterations);

    // Verify the correct size. There are initially 24 tetrahedra that each
    // split into 8 sub tetrahedra.
    const size_t num_tetrahedra = 24 * std::pow(8, level);
    EXPECT_EQ(mesh.tetrahedra().size(), num_tetrahedra);

    // Verify that the volume monotonically converges towards the exact volume
    // of the given level 0 cylinder.
    const double volume = CalcTetrahedronMeshVolume(mesh);
    const double error = cylinder_volume - volume;

    EXPECT_GT(error, 0.0);
    EXPECT_LT(error, prev_error);

    // Always compare against last computed error to show monotonic convergence.
    prev_error = error;
  }
}

// This could use a better hash function, but two random primes do pretty well
struct EdgeHashFunction {
  std::size_t operator()(const std::pair<VolumeVertexIndex,
      VolumeVertexIndex>& element) const {
    // Take the convention that x is the smaller of the pair
    // so that the hash is commutative
    const VolumeVertexIndex x = std::min(element.first, element.second);
    const VolumeVertexIndex y = std::max(element.first, element.second);

    return x * 779230947 + y * 247091631;
  }
};

struct EdgeEqualsFunction {
  bool operator()(const std::pair<VolumeVertexIndex, VolumeVertexIndex>& a,
                  const std::pair<VolumeVertexIndex, VolumeVertexIndex>& b)
                  const {
    VolumeVertexIndex ax = std::min(a.first, a.second);
    VolumeVertexIndex ay = std::max(a.first, a.second);
    VolumeVertexIndex bx = std::min(b.first, b.second);
    VolumeVertexIndex by = std::max(b.first, b.second);

    return (ax == bx) && (ay == by);
  }
};

// Counts the unique 1-simplices in the mesh
int CountEdges(const VolumeMesh<double>& mesh) {
  std::unordered_set<std::pair<VolumeVertexIndex, VolumeVertexIndex>,
      EdgeHashFunction, EdgeEqualsFunction> edges;

  for (auto &t : mesh.tetrahedra()) {
    // 6 edges of a tetrahedron
    edges.insert(std::make_pair(t.vertex(0), t.vertex(1)));
    edges.insert(std::make_pair(t.vertex(1), t.vertex(2)));
    edges.insert(std::make_pair(t.vertex(0), t.vertex(2)));
    edges.insert(std::make_pair(t.vertex(0), t.vertex(3)));
    edges.insert(std::make_pair(t.vertex(1), t.vertex(3)));
    edges.insert(std::make_pair(t.vertex(2), t.vertex(3)));
  }
  return edges.size();
}

// Sort the vertex indices so that abc == cba == acb == etc...
struct FaceHashFunction {
  std::size_t operator()(
      const std::tuple<VolumeVertexIndex,
                       VolumeVertexIndex,
                       VolumeVertexIndex>& element) const {
    // Take the convention that x is the smaller of the
    // pair so that the hash is commutative
    std::vector<VolumeVertexIndex> v =
        {std::get<0>(element), std::get<1>(element), std::get<2>(element)};
    std::sort(v.begin(), v.end());
    const VolumeVertexIndex x = v[0];
    const VolumeVertexIndex y = v[1];
    const VolumeVertexIndex z = v[2];

    return x * 779230947 + y * 247091631 + z * 119428962;
  }
};

struct FaceEqualsFunction {
  bool operator() (
      const std::tuple<VolumeVertexIndex, VolumeVertexIndex,
      VolumeVertexIndex>& a,
      const std::tuple<VolumeVertexIndex, VolumeVertexIndex,
      VolumeVertexIndex>& b) const {
    std::vector<VolumeVertexIndex> av =
        {std::get<0>(a), std::get<1>(a), std::get<2>(a)};

    std::vector<VolumeVertexIndex> bv =
        {std::get<0>(b), std::get<1>(b), std::get<2>(b)};

    std::sort(av.begin(), av.end());
    std::sort(bv.begin(), bv.end());

    const VolumeVertexIndex ax = av[0], ay = av[1], az = av[2];
    const VolumeVertexIndex bx = bv[0], by = bv[1], bz = bv[2];

    return (ax == bx) && (ay == by) && (az == bz);
  }
};

// Counts the unique 2-simplices in the face
int CountFaces(const VolumeMesh<double>& mesh) {
  std::unordered_set<
      std::tuple<VolumeVertexIndex, VolumeVertexIndex, VolumeVertexIndex>,
      FaceHashFunction,
      FaceEqualsFunction> faces;

  for (const auto & t : mesh.tetrahedra()) {
    // 4 faces of a tetrahedron, all facing in
    faces.insert(std::make_tuple(t.vertex(0), t.vertex(1), t.vertex(2)));
    faces.insert(std::make_tuple(t.vertex(1), t.vertex(0), t.vertex(3)));
    faces.insert(std::make_tuple(t.vertex(2), t.vertex(1), t.vertex(3)));
    faces.insert(std::make_tuple(t.vertex(0), t.vertex(2), t.vertex(3)));
  }

  return faces.size();
}

int ComputeEulerCharacteristic(const VolumeMesh<double>& mesh) {
  const int k0 = mesh.vertices().size();
  const int k1 = CountEdges(mesh);
  const int k2 = CountFaces(mesh);
  const int k3 = mesh.tetrahedra().size();

  return k0 - k1 + k2 - k3;
}

// Looking at the tetrahedral mesh as a convex 4 dimensional
// simplicial complex This test computes the generalized euler characteristic
//
// χ = k_0 - k_1 + k_2 - k_3
//
// where k_i is the number of i-simplexes in the complex. For a convex mesh that
// is homeomorphic to a 3 dimensional ball, χ = 1

GTEST_TEST(MakeCylinderMesh, EulerCharacteristic) {
  const double height = 2;
  const double radius = 1;
  const int smoothing_iterations = 0;

  const int expected_euler_characteristic = 1;

  for (int level = 0; level < 6; ++level) {
    auto mesh = MakeCylinderMesh<double>(height, radius,
        level, smoothing_iterations);

    EXPECT_EQ(ComputeEulerCharacteristic(mesh), expected_euler_characteristic);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
