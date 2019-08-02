#include "drake/geometry/proximity/make_box_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// TODO(DamrongGuoy): Switch to something like boost::multi_array.
//  We like "int vertex_index[nx][ny][nz]"; however, it loses info at
//  function call boundaries. Furthermore, we calculate nx, ny, nz at run
//  time, so we can't "typedef int MultiArrayInt3D[nx][ny][nz]".
//  Here we use a less-efficient work around by nested std::vector.
typedef std::vector<int> MultiArrayInt1D;
typedef std::vector<std::vector<int>> MultiArrayInt2D;
typedef std::vector<std::vector<std::vector<int>>> MultiArrayInt3D;

// For testing purpose, we inherit protected functions from MakeBoxVolumeMesh
// to make them publicly accessible in our unit tests.
template <typename T>
class MakeBoxVolumeMeshTester : public MakeBoxVolumeMesh<T> {
 public:
  static std::vector<T> UniformSample(T first, T last, int num_sample) {
    return MakeBoxVolumeMesh<T>::UniformSample(first, last, num_sample);
  }

  static std::vector<VolumeVertex<T>> GenerateVertices(
      const Box& box, const Vector3<int>& num_vertex,
      MultiArrayInt3D* vertex_index) {
    return MakeBoxVolumeMesh<T>::GenerateVertices(box, num_vertex,
                                                  vertex_index);
  }

  static void AddSixTetrahedraOfCell(const Vector3<int>& base,
                                     const MultiArrayInt3D& vertex_index,
                                     std::vector<VolumeElement>* elements) {
    MakeBoxVolumeMesh<T>::AddSixTetrahedraOfCell(base, vertex_index, elements);
  }

  static std::vector<VolumeElement> GenerateElements(
      const Vector3<int>& num_vertex, const MultiArrayInt3D& vertex_index) {
    return MakeBoxVolumeMesh<T>::GenerateElements(num_vertex, vertex_index);
  }
};

GTEST_TEST(MakeBoxVolumeMeshTest, UniformSample) {
  const double first = -0.5;
  const double last = 0.5;
  const int num_sample = 3;
  std::vector<double> samples =
      MakeBoxVolumeMeshTester<double>::UniformSample(first, last, num_sample);

  EXPECT_EQ(3, samples.size());
  EXPECT_EQ(-0.5, samples[0]);
  EXPECT_EQ(0.0, samples[1]);
  EXPECT_EQ(0.5, samples[2]);
}

GTEST_TEST(MakeBoxVolumeMeshTest, GenerateVertices) {
  // Set up a box [-1,1]x[-2,2]x[-3,3] whose corners have integer coordinates.
  const Box box(2.0, 4.0, 6.0);
  // Request the number of vertices so that the vertices have integer
  // coordinates {-1,0,1} x {-2,-1,0,1,2} x {-3,-2,-1,0,1,2,3}.
  // Vertex[i][j][k] should have coordinates (i-1, j-2, k-3) for
  // 0 ≤ i < 3, 0 ≤ j < 5, 0 ≤ k < 7.
  const Vector3<int> num_vertex{3, 5, 7};
  MultiArrayInt3D vertex_index(
      num_vertex.x(),
      MultiArrayInt2D(num_vertex.y(), MultiArrayInt1D(num_vertex.z())));

  auto vertices = MakeBoxVolumeMeshTester<double>::GenerateVertices(
      box, num_vertex, &vertex_index);

  EXPECT_EQ(105, vertices.size());
  for (int i = 0; i < num_vertex.x(); ++i) {
    for (int j = 0; j < num_vertex.y(); ++j) {
      for (int k = 0; k < num_vertex.z(); ++k) {
        int sequential_index = vertex_index[i][j][k];
        Vector3<double> expect_r_MV = Vector3<double>(i - 1, j - 2, k - 3);
        auto r_MV = vertices[sequential_index].r_MV();
        EXPECT_TRUE(CompareMatrices(expect_r_MV, r_MV))
                    << "Incorrect vertex position.";
      }
    }
  }
}

GTEST_TEST(MakeBoxVolumeMeshTest, AddSixTetrahedraOfCell2) {
  const Vector3<int> lowest(1, 2, 3);
  MultiArrayInt3D vertex_index(3, MultiArrayInt2D(4, MultiArrayInt1D(5)));
  int index = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 4; ++j)
      for (int k = 0; k < 5; ++k)
        vertex_index[i][j][k] = index++;
  std::vector<VolumeElement> elements;

  MakeBoxVolumeMeshTester<double>::AddSixTetrahedraOfCell(lowest, vertex_index,
                                                          &elements);
  ASSERT_EQ(6, elements.size());

  // In a 3x4x5 grid of vertices. vertex_index[1][2][3] = 33. This picture
  // shows how the rectangular cell with its lowest vertex v₃₃ looks like.
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
  const Vector3<int>& num_vertex{3, 5, 7};
  const int expect_total_num_vertex =
      num_vertex.x() * num_vertex.y() * num_vertex.z();

  const Vector3<int>& num_cell{2, 4, 6};
  const int expect_num_cell = num_cell.x() * num_cell.y() * num_cell.z();
  const int expect_num_element = 6 * expect_num_cell;

  MultiArrayInt3D vertex_index(3, MultiArrayInt2D(5, MultiArrayInt1D(7)));
  int index = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 5; ++j)
      for (int k = 0; k < 7; ++k)
        vertex_index[i][j][k] = index++;

  auto elements = MakeBoxVolumeMeshTester<double>::GenerateElements(
      num_vertex, vertex_index);

  EXPECT_EQ(expect_num_element, elements.size());
  // TODO(DamrongGuoy): Find a better way to test `elements`. Currently we
  //  only test that each tetrahedron uses vertices with indices in the range
  //  [0, expect_total_num_vertex).
  for (const auto& tetrahedron : elements) {
    for (int v = 0; v < 4; ++v) {
      EXPECT_GE(tetrahedron.vertex(v), 0);
      EXPECT_LT(tetrahedron.vertex(v), expect_total_num_vertex);
    }
  }
}

// TODO(DamrongGuoy): Move the following two functions to the appropriate
//  place, so they can be shared by both make_unit_sphere_mesh_test.cc and
//  make_box_mesh_test.cc.  Right now they are duplicated.

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
  const std::vector<VolumeVertex<double>>& vertices = mesh.vertices();
  const std::vector<VolumeElement>& tetrahedra = mesh.tetrahedra();
  double volume = 0.0;
  for (const auto& t : tetrahedra) {
    double tetrahedron_volume = CalcTetrahedronVolume(
        vertices[t.vertex(0)].r_MV(), vertices[t.vertex(1)].r_MV(),
        vertices[t.vertex(2)].r_MV(), vertices[t.vertex(3)].r_MV());
    EXPECT_GT(tetrahedron_volume, 0.0);
    volume += tetrahedron_volume;
  }
  return volume;
}

GTEST_TEST(MakeBoxVolumeMeshTest, GenerateMesh) {
  const Box box(0.2, 0.4, 0.8);
  VolumeMesh<double> box_mesh = MakeBoxVolumeMesh<double>::Generate(box, 0.1);

  const int rectangular_cells = 2 * 4 * 8;
  const int tetrahedra_per_cell = 6;
  const int expect_num_tetrahedra = rectangular_cells * tetrahedra_per_cell;
  EXPECT_EQ(expect_num_tetrahedra, box_mesh.num_elements());

  const int expect_num_vertices = 3 * 5 * 9;
  EXPECT_EQ(expect_num_vertices, box_mesh.num_vertices());

  const double expect_volume = box.width() * box.depth() * box.height();
  EXPECT_NEAR(expect_volume, CalcTetrahedronMeshVolume(box_mesh),
              2.0 * std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
