#include "drake/geometry/proximity/make_sphere_mesh.h"

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// TODO(amcastro-tri): The unit tests below were designed with the idea in mind
// that if a typical bug is present (for instance wrong tetrahedron sign
// convention, missing tetrahedra or faulty vertices computation) they would
// fail. However, from a purist point of view on unit testing, a variety of
// tests are missing. To mention a few:
//   1. Level 0 tessellation should have all of its properties confirmed:
//      tetrahedra sign convention, positive volume, sphere coverage.
//   2. Is level 0 correct in that there are no overlapping tetrahedra?
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

// Loosely confirms that there are no duplicate vertices in the sphere mesh. In
// this case, we examine the first four levels of refinement (0, 1, 2, 3) and
// compare vertex count against the documented expected vertex count in
// [Everett, 1997]. This isn't proof, but it's highly suggestive.
GTEST_TEST(MakeSphereMesh, NoDuplicateVertices) {
  // Expected number of vertices based on [Everett, 1997].
  const std::vector<int> expected_v_count{7, 25, 129, 833};
  for (int i = 0; i < 4; ++i) {
    VolumeMesh<double> mesh = MakeUnitSphereMesh<double>(i);
    EXPECT_EQ(mesh.num_vertices(), expected_v_count[i]);
    EXPECT_EQ(mesh.num_elements(), std::pow(8, i + 1));
  }
  // TODO(SeanCurtis-TRI): Actually test for duplicates within a distance
  //  threshold.
}

// Smoke test to confirm the calculations work with the Autodiff scalar.
GTEST_TEST(MakeSphereMesh, AutoDiffRefinement) {
  VolumeMesh<AutoDiffXd> mesh1 = MakeUnitSphereMesh<AutoDiffXd>(1);
  EXPECT_EQ(mesh1.num_vertices(), 25);
  EXPECT_EQ(mesh1.num_elements(), 64);
}

// Confirms that the SplitOctohedron function splits in the expected direction.
GTEST_TEST(MakeSphereMesh, SplitOctohedron) {
  // Define a tet such that when we divide all the edges in half the octohedron
  // remaining is the canonical octohedron with vertices at (±1, 0, 0),
  // (0, ±1, 0), and (0, 0, ±1). We do this so there's no question about the
  // semantics of vertices e-j as defined in the make unit sphere
  // infrastructure.
  const int a(0), b(1), c(2), d(3);
  const std::vector<VolumeVertex<double>> tet_vertices{
      VolumeVertex<double>(1, -1, -1), VolumeVertex<double>(1, 1, 1),
      VolumeVertex<double>(-1, 1, -1), VolumeVertex<double>(-1, -1, 1)};

  auto mid_point = [&tet_vertices](int i, int j) -> Vector3d {
    return (tet_vertices[i].r_MV() + tet_vertices[j].r_MV()) / 2;
  };

  using VIndex = VolumeVertexIndex;
  // We don't need the vertices for a, b, c, and d in the test; we just need
  // e-j.
  const VIndex e(0), f(1), g(2), h(3), i(4), j(5);
  const std::array<VIndex, 6> octo_vertices{e, f, g, h, i, j};
  const std::vector<VolumeVertex<double>> unit_p_MVs{
      VolumeVertex<double>(mid_point(a, b)),  // e = (a + b) / 2
      VolumeVertex<double>(mid_point(a, c)),  // f = (a + c) / 2
      VolumeVertex<double>(mid_point(a, d)),  // g = (a + d) / 2
      VolumeVertex<double>(mid_point(b, c)),  // h = (b + c) / 2
      VolumeVertex<double>(mid_point(b, d)),  // i = (b + d) / 2
      VolumeVertex<double>(mid_point(c, d))};  // j = (c + d) / 2

  // Confirm the tetrahedron works as advertised; that the vertices at edge
  // midpoints live where we expect them to.
  //          +Z
  //          |  /
  //          i j
  //          |/
  //   ---g---+---h---+Y
  //         /|
  //        e f
  //       /  |
  //     +X
  ASSERT_TRUE(CompareMatrices(unit_p_MVs[e].r_MV(), Vector3d(1, 0, 0)));
  ASSERT_TRUE(CompareMatrices(unit_p_MVs[f].r_MV(), Vector3d(0, 0, -1)));
  ASSERT_TRUE(CompareMatrices(unit_p_MVs[g].r_MV(), Vector3d(0, -1, 0)));
  ASSERT_TRUE(CompareMatrices(unit_p_MVs[h].r_MV(), Vector3d(0, 1, 0)));
  ASSERT_TRUE(CompareMatrices(unit_p_MVs[i].r_MV(), Vector3d(0, 0, 1)));
  ASSERT_TRUE(CompareMatrices(unit_p_MVs[j].r_MV(), Vector3d(-1, 0, 0)));

  // We're going to implicitly scale the tetrahedron so that the octohedron
  // is no longer symmetric; we scale it along the three axes by a factor of
  // 1, 2, and 2. The axis under test is scaled by 1 and the other axes by 2.
  // SplitOctohedron() will favor splitting along the shortest axis. In the
  // tests above, it's clear that EJ is aligned with the x-axis, GH with the y,
  // and FI with the z.
  const std::vector<SortedPair<VIndex>> split_edges{SortedPair<VIndex>{e, j},
                                                    SortedPair<VIndex>{g, h},
                                                    SortedPair<VIndex>{f, i}};
  for (int axis = 0; axis < 3; ++axis) {
    Vector3d scale_factor{2, 2, 2};
    scale_factor(axis) = 1;

    std::vector<VolumeVertex<double>> p_MVs;
    std::transform(
        unit_p_MVs.begin(), unit_p_MVs.end(), std::back_inserter(p_MVs),
        [&scale_factor](const VolumeVertex<double>& v) {
          return VolumeVertex<double>(v.r_MV().cwiseProduct(scale_factor));
        });
    std::vector<VolumeElement> split_tetrahedra;

    SplitOctohedron(octo_vertices, p_MVs, &split_tetrahedra);

    EXPECT_EQ(split_tetrahedra.size(), 4u);
    // Confirm that every tetrahedron has the expected split edge and *doesn't*
    // have the other split edges.
    int split_count[3] = {0, 0, 0};
    for (const auto& tet : split_tetrahedra) {
      for (int v = 0; v < 3; ++v) {
        for (int u = v + 1; u < 4; ++u) {
          SortedPair<VIndex> test_edge(tet.vertex(v), tet.vertex(u));
          for (int ax = 0; ax < 3; ++ax) {
            if (test_edge == split_edges[ax]) ++split_count[ax];
          }
        }
      }
    }
    EXPECT_EQ(split_count[axis], 4);
    EXPECT_EQ(split_count[(axis + 1) % 3], 0);
    EXPECT_EQ(split_count[(axis + 2) % 3], 0);
  }
}

// Confirms that for the given edge length, that the resulting sphere mesh's
// equator edge length is at or below the specified edge length.
GTEST_TEST(MakeSphereVolumeMesh, ConfirmEdgeLength) {
  const double r = 1.5;
  const double r_squared = r * r;
  Sphere sphere(r);
  auto test_equator = [r_squared](const VolumeMesh<double>& mesh,
                                  double edge_length) {
    // We arbitrarily pick the z = 0 equator (although x = 0 or y = 0 would work
    // equally well).
    std::vector<Vector3d> equator_vertices;
    for (const auto& vertex : mesh.vertices()) {
      const Vector3d& p_MV = vertex.r_MV();
      const double d_squared = p_MV(0) * p_MV(0) + p_MV(1) * p_MV(1);
      if (p_MV(2) == 0.0 && d_squared >= r_squared - 1e-14) {
        equator_vertices.push_back(p_MV);
      }
    }
    // Sort the vertices according to radial angle - we rely on the algorithm
    // producing strictly unique vertices (no duplicates).
    std::sort(equator_vertices.begin(), equator_vertices.end(),
              [](const auto& v1, const auto& v2) {
                const double theta1 = std::atan2(v1(1), v1(0));
                const double theta2 = std::atan2(v2(1), v2(0));
                return theta1 < theta2;
              });

    // The number of equator vertices should be 4 * 2^L. However, we don't know
    // a priori what L is. So, we infer L from the vertex count and confirm that
    // it produces the observed vertex count.
    const int v_count = static_cast<int>(equator_vertices.size());
    EXPECT_GE(v_count, 4);
    const int apparent_L = static_cast<int>(std::log2(v_count / 4));
    EXPECT_EQ(4.0 * std::pow(2.0, apparent_L), v_count);

    // Given radial ordering of vertices, confirm that the edge between two
    // sequential vertices does not exceed the target edge_length.
    for (int i = 0; i < v_count; ++i) {
      const double chord_length =
          (equator_vertices[i] - equator_vertices[(i + 1) % v_count]).norm();
      ASSERT_LE(chord_length, edge_length);
    }
  };

  // Note: at level zero, the longest edge lengths are sqrt(2) * r. So, we pick
  // a length slightly longer (1.5 * r) to test the base case. After that simply
  // values that decrease by half. We also confirm tet count increases when we
  // cut things in half.
  int previous_tet_count = 0;
  for (const double edge_length : {1.5 * r, r, r / 2, r / 4}) {
    VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(sphere, edge_length);
    int current_tet_count = mesh.num_elements();
    EXPECT_LT(previous_tet_count, current_tet_count);
    previous_tet_count = current_tet_count;
    test_equator(mesh, edge_length);
  }
}

// Confirms that edge length larger than sphere diameter still produces the
// octohedron.
GTEST_TEST(MakeSphereVolumeMesh, MassiveEdgeLength) {
  const Sphere sphere(1.5);
  const double edge_length = 3 * sphere.radius();
  VolumeMesh<double> mesh = MakeSphereVolumeMesh<double>(sphere, edge_length);
  EXPECT_EQ(mesh.num_elements(), 8);
  EXPECT_EQ(mesh.num_vertices(), 7);
}

// Smoke test only. Assume correctness of MakeSphereVolumeMesh() and
// ConvertVolumeToSurfaceMesh(). The edge length larger than sphere diameter
// produces an octahedron with 8 triangles and 6 vertices.
GTEST_TEST(MakeSphereSurfaceMesh, GenerateSurface) {
  const Sphere sphere(1.5);
  const double edge_length = 3 * sphere.radius();
  SurfaceMesh<double> surface_mesh =
      MakeSphereSurfaceMesh<double>(sphere, edge_length);
  EXPECT_EQ(surface_mesh.num_faces(), 8);
  EXPECT_EQ(surface_mesh.num_vertices(), 6);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
