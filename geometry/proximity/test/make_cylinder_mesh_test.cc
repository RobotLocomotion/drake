#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <algorithm>
#include <fstream>
#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// TODO(DamrongGuoy): Move this function to VolumeMesh.

// Computes the total volume of a VolumeMesh by summing up the contribution
// of each tetrahedron.
double CalcTetrahedronMeshVolume(const VolumeMesh<double>& mesh) {
  double volume = 0.0;
  for (int e = 0; e < mesh.num_elements(); e++) {
    volume += mesh.CalcTetrahedronVolume(VolumeElementIndex(e));
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

  auto mesh0 = MakeCylinderMesh<double>(
      drake::geometry::Cylinder(radius, height), refinement_level);

  const double volume0 = CalcTetrahedronMeshVolume(mesh0);
  const double cylinder_volume = height * radius * radius * M_PI;

  // Initial error in the computation of the volume.
  double prev_error = cylinder_volume - volume0;

  // The volume of a rectangular prism with base
  // of size l x w and height h is V = lwh.
  // For our level zero cylinder, we have a prism of height 2 and
  // l = w = sqrt(2) and height = 2.0. Thus its volume is 4.
  const double expected_volume_0 = 4.0;

  EXPECT_NEAR(volume0, expected_volume_0, kTolerance);

  for (int level = 1; level < 6; ++level) {
    auto mesh = MakeCylinderMesh<double>(
        drake::geometry::Cylinder(radius, height), level);

    // Verify the correct size. There are initially 24 tetrahedra that each
    // split into 8 sub tetrahedra.
    const size_t num_tetrahedra = 24 * std::pow(8, level);
    EXPECT_EQ(mesh.tetrahedra().size(), num_tetrahedra);

    // Verify that the volume monotonically converges towards the exact volume
    // of the given cylinder.
    const double volume = CalcTetrahedronMeshVolume(mesh);
    const double error = cylinder_volume - volume;

    EXPECT_GT(error, 0.0);
    EXPECT_LT(error, prev_error);

    // Always compare against last computed error to show monotonic convergence.
    prev_error = error;
  }
}

// Counts the unique 1-simplices in the mesh
int CountEdges(const VolumeMesh<double>& mesh) {
  std::unordered_set<SortedPair<VolumeVertexIndex>> edges;

  for (auto& t : mesh.tetrahedra()) {
    // 6 edges of a tetrahedron
    edges.insert(MakeSortedPair(t.vertex(0), t.vertex(1)));
    edges.insert(MakeSortedPair(t.vertex(1), t.vertex(2)));
    edges.insert(MakeSortedPair(t.vertex(0), t.vertex(2)));
    edges.insert(MakeSortedPair(t.vertex(0), t.vertex(3)));
    edges.insert(MakeSortedPair(t.vertex(1), t.vertex(3)));
    edges.insert(MakeSortedPair(t.vertex(2), t.vertex(3)));
  }
  return edges.size();
}

// Counts the unique 2-simplices in the face
int CountFaces(const VolumeMesh<double>& mesh) {
  std::set<SortedTriplet<VolumeVertexIndex>> faces;

  for (const auto& t : mesh.tetrahedra()) {
    // 4 faces of a tetrahedron, all facing in
    faces.insert(SortedTriplet<VolumeVertexIndex>(t.vertex(0), t.vertex(1),
                                                  t.vertex(2)));
    faces.insert(SortedTriplet<VolumeVertexIndex>(t.vertex(1), t.vertex(0),
                                                  t.vertex(3)));
    faces.insert(SortedTriplet<VolumeVertexIndex>(t.vertex(2), t.vertex(1),
                                                  t.vertex(3)));
    faces.insert(SortedTriplet<VolumeVertexIndex>(t.vertex(0), t.vertex(2),
                                                  t.vertex(3)));
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

// Confirm that the mesh is well formed (i.e., with no duplicate vertices or
// tetrahedra) by computing its Euler characteristic. Looking at the
// tetrahedral mesh as a convex 4 dimensional simplicial complex, this test
// computes the generalized euler characteristic:
//
// χ = k_0 - k_1 + k_2 - k_3
//
// where k_i is the number of i-simplexes in the complex. For a convex mesh that
// is homeomorphic to a 3 dimensional ball, χ = 1.
GTEST_TEST(MakeCylinderMesh, EulerCharacteristic) {
  const double height = 2;
  const double radius = 1;

  const int expected_euler_characteristic = 1;

  for (int level = 0; level < 6; ++level) {
    auto mesh = MakeCylinderMesh<double>(
        drake::geometry::Cylinder(radius, height), level);

    EXPECT_EQ(ComputeEulerCharacteristic(mesh), expected_euler_characteristic);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
