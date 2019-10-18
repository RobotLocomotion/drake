#include "drake/geometry/proximity/make_box_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MakeBoxVolumeMeshTest, CalcSequentialIndex) {
  const Vector3<int> num_vertices(3, 2, 5);
  EXPECT_EQ(28, CalcSequentialIndex(2, 1, 3, num_vertices));
}

GTEST_TEST(MakeBoxVolumeMeshTest, GenerateVertices) {
  // Set up a box [-1,1]x[-2,2]x[-3,3] whose corners have integer coordinates.
  const Box box(2.0, 4.0, 6.0);
  // Request the number of vertices so that the vertices have integer
  // coordinates {-1,0,1} x {-2,-1,0,1,2} x {-3,-2,-1,0,1,2,3}.
  // Vertex[i][j][k] should have coordinates (i-1, j-2, k-3) for
  // 0 ≤ i < 3, 0 ≤ j < 5, 0 ≤ k < 7.
  const Vector3<int> num_vertices{3, 5, 7};

  auto vertices = GenerateVertices<double>(box, num_vertices);

  EXPECT_EQ(105, vertices.size());
  for (int i = 0; i < num_vertices.x(); ++i) {
    for (int j = 0; j < num_vertices.y(); ++j) {
      for (int k = 0; k < num_vertices.z(); ++k) {
        int sequential_index = CalcSequentialIndex(i, j, k, num_vertices);
        Vector3<double> expect_r_MV = Vector3<double>(i - 1, j - 2, k - 3);
        Vector3<double> r_MV = vertices[sequential_index].r_MV();
        EXPECT_TRUE(CompareMatrices(expect_r_MV, r_MV))
                    << "Incorrect vertex position.";
      }
    }
  }
}

GTEST_TEST(MakeBoxVolumeMeshTest, AddSixTetrahedraOfCell) {
  const Vector3<int> lowest(1, 2, 3);
  const Vector3<int> num_vertices(3, 4, 5);
  std::vector<VolumeElement> elements;

  AddSixTetrahedraOfCell(lowest, num_vertices, &elements);
  ASSERT_EQ(6, elements.size());

  // In a 3x4x5 grid of vertices, the vertex with (i,j,k)-index = (1,2,3) has
  // its sequential index 33. This picture shows how the rectangular cell
  // with its lowest vertex v₃₃ looks like.
  //
  //               v₃₄     v₃₉
  //               ●------●
  //              /|     /|
  //             / | v₅₉/ |
  //        v₅₄ ●------●  |
  //            |  |   |  |
  //            |  ●---|--● v₃₈
  //            | /v₃₃ | /
  //            |/     |/
  //    +K  v₅₃ ●------● v₅₈
  //     |
  //     |
  //     o------+J
  //    /
  //   /
  // +I
  //
  // This table has the expected six tetrahedra of the rectangular cell.
  // They share the main diagonal v₃₃v₅₉.
  const int expect_elements[6][4] {
      // clang-format off
      {33, 59, 53, 58},
      {33, 59, 58, 38},
      {33, 59, 38, 39},
      {33, 59, 39, 34},
      {33, 59, 34, 54},
      {33, 59, 54, 53}};
  // clang-format on
  for (int e = 0; e < 6; ++e)
    for (int v = 0; v < 4; ++v)
      EXPECT_EQ(expect_elements[e][v], elements[e].vertex(v));
}

GTEST_TEST(MakeBoxVolumeMeshTest, GenerateElements) {
  const Vector3<int> num_vertices{3, 5, 7};
  const int expect_total_num_vertex =
      num_vertices.x() * num_vertices.y() * num_vertices.z();

  const Vector3<int> num_cell = num_vertices - Vector3<int>::Ones();
  const int expect_num_cell = num_cell.x() * num_cell.y() * num_cell.z();
  const int expect_num_element = 6 * expect_num_cell;

  auto elements = GenerateElements(num_vertices);

  EXPECT_EQ(expect_num_element, elements.size());
  // TODO(DamrongGuoy): Find a better way to test `elements`. Currently we
  //  only test that each tetrahedron uses vertices with indices in the range
  //  [0, expect_total_num_vertex). Perhaps check Euler characteristic,
  //  i.e., #vertex - #edge + #triangle - #tetrahedron = 1.
  for (const auto& tetrahedron : elements) {
    for (int v = 0; v < 4; ++v) {
      EXPECT_GE(tetrahedron.vertex(v), 0);
      EXPECT_LT(tetrahedron.vertex(v), expect_total_num_vertex);
    }
  }
}

GTEST_TEST(MakeBoxVolumeMeshTest, GenerateMesh) {
  const Box box(0.2, 0.4, 0.8);
  VolumeMesh<double> box_mesh = MakeBoxVolumeMesh<double>(box, 0.1);

  const int rectangular_cells = 2 * 4 * 8;
  const int tetrahedra_per_cell = 6;
  const int expect_num_tetrahedra = rectangular_cells * tetrahedra_per_cell;
  EXPECT_EQ(expect_num_tetrahedra, box_mesh.num_elements());

  const int expect_num_vertices = 3 * 5 * 9;
  EXPECT_EQ(expect_num_vertices, box_mesh.num_vertices());

  const double expect_volume = box.width() * box.depth() * box.height();
  double volume = 0.0;
  for (int e = 0; e < box_mesh.num_elements(); ++e) {
    double tetrahedron_volume =
        box_mesh.CalcTetrahedronVolume(VolumeElementIndex(e));
    EXPECT_GT(tetrahedron_volume, 0.0);
    volume += tetrahedron_volume;
  }
  EXPECT_NEAR(expect_volume, volume,
              2.0 * std::numeric_limits<double>::epsilon());
}

// Smoke test only. Assume correctness of MakeBoxVolumeMesh() and
// ConvertVolumeToSurfaceMesh().
GTEST_TEST(MakeBoxSurfaceMeshTest, GenerateSurface) {
  const Box box(0.2, 0.4, 0.8);
  const double target_edge_length = 0.1;
  SurfaceMesh<double> surface_mesh =
      MakeBoxSurfaceMesh<double>(box, target_edge_length);

  const int expect_num_vertices = 114;
  EXPECT_EQ(expect_num_vertices, surface_mesh.num_vertices());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
