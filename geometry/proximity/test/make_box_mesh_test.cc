#include "drake/geometry/proximity/make_box_mesh.h"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

using Eigen::Vector3d;
using std::abs;
using std::find;
using std::find_if;
using std::for_each;

// TODO(#14235): Move IsTetrahedronRespectingMa() to possibly
//  proximity_utilities and extend it to other shape primitives.

// Returns true if `tetrahedron` in `mesh` has its four vertices belonging to
// the same block of the medial-axis subdivision of `box`, i.e., there is a
// box's face to which all four vertices are _closest_. Here a vertex is
// _closest_ to a box's face when there is no other closer face. Notice that
// a vertex may have multiple closest faces, i.e., it is on the medial axis.
//
// We use the given `tolerance` for comparing distances.
//
// @pre No vertex of the tetrahedron is outside the box.
//
// Here we omit the frame notations because everything is expressed in the
// frame of the box. It might change in the future.
bool IsTetrahedronRespectingMa(const VolumeElement& tetrahedron,
                               const VolumeMesh<double>& mesh, const Box& box,
                               const double tolerance) {
  const Vector3d half_size = box.size() / 2.;
  auto distance_to_boundary = [&half_size](const Vector3d& point_in_box) {
    Vector3d abs_point = point_in_box.array().abs();
    // Verify that the point is in the box.
    DRAKE_DEMAND((half_size.array() >= abs_point.array()).all());
    return (half_size - abs_point).minCoeff();
  };
  // The six faces of the box are the ±X, ±Y, ±Z faces.
  for (int axis = 0; axis < 3; ++axis) {
    for (const double face_bound : {-half_size[axis], half_size[axis]}) {
      bool closest_to_this_face = true;
      for (int i = 0; i < mesh.kVertexPerElement && closest_to_this_face; ++i) {
        const Vector3d vertex = mesh.vertex(tetrahedron.vertex(i));
        const double distance_to_this_face = abs(vertex[axis] - face_bound);
        closest_to_this_face =
            tolerance >= distance_to_this_face - distance_to_boundary(vertex);
      }
      if (closest_to_this_face) {
        return true;
      }
    }
  }
  return false;
}

// Verifies that a tetrahedral mesh of a box from MakeBoxVolumeMeshWithMa()
// satisfies all these properties:
//
//   A. The mesh is conforming.
//   B. The mesh conforms to the box.
//   C. The mesh conforms to the box's medial axis.
//
// We will use this function in all unit tests of MakeBoxVolumeMeshWithMa().
//
// @retval true when all conditions passed.
//
// Here we omit the frame notations because everything is expressed in the
// frame of the box. It might change in the future.
bool VerifyBoxMeshWithMa(const VolumeMesh<double>& mesh, const Box& box) {
  // A. The mesh is conforming.
  // A1. The mesh has unique vertices.
  const int num_vertices = mesh.num_vertices();
  for (int i = 0; i < num_vertices; ++i) {
    for (int j = i + 1; j < num_vertices; ++j) {
      const bool vertex_is_unique = mesh.vertex(i) != mesh.vertex(j);
      EXPECT_TRUE(vertex_is_unique) << "The mesh has duplicated vertices.";
      if (!vertex_is_unique) {
        return false;
      }
    }
  }
  // A2. Euler characteristic = 1. This is a necessary condition (but may not
  //     be sufficient) for conforming tetrahedra (two tetrahedra intersect in
  //     their shared face, or shared edge, or shared vertex, or not at all.
  //     There is no partial overlapping of two tetrahedra.).
  const int euler_characteristic = ComputeEulerCharacteristic(mesh);
  EXPECT_EQ(1, euler_characteristic)
      << "The mesh's tetrahedra are not conforming because the mesh's Euler "
         "characteristic is "
      << euler_characteristic << " instead of 1.";
  if (euler_characteristic != 1) {
    return false;
  }

  // B. The mesh conforms to the box.
  // B1. The mesh has a vertex at each of the 8 corner points of the box.
  const Vector3d half_size = box.size() / 2.;
  for (const double x : {-half_size.x(), half_size.x()}) {
    for (const double y : {-half_size.y(), half_size.y()}) {
      for (const double z : {-half_size.z(), half_size.z()}) {
        const Vector3d corner(x, y, z);
        const bool corner_is_a_mesh_vertex =
            mesh.vertices().end() !=
            find_if(mesh.vertices().begin(), mesh.vertices().end(),
                    [&corner](const Vector3d& v) -> bool {
                      return v == corner;
                    });
        EXPECT_TRUE(corner_is_a_mesh_vertex)
            << "A corner point of the box is missing from the mesh vertices.";
        if (!corner_is_a_mesh_vertex) {
          return false;
        }
      }
    }
  }
  // B2. No mesh's vertex is outside the box.
  for (int i = 0; i < num_vertices; ++i) {
    const bool vertex_is_inside_or_on_boundary =
        (mesh.vertex(i).array().abs() <= half_size.array()).all();
    EXPECT_TRUE(vertex_is_inside_or_on_boundary)
        << "A mesh vertex is outside the box.";
    if (!vertex_is_inside_or_on_boundary) {
      return false;
    }
  }
  // B3. The volume of the mesh equals the volume of the box.
  const double mesh_volume = mesh.CalcVolume();
  const double box_volume = box.width() * box.depth() * box.height();
  // We estimate the tolerance from:
  // 1. There are about 4 multiply+add in calculating a tetrahedron's volume,
  //    so we have 8 epsilons.
  // 2. The rounding errors accumulate over the number of tetrahedra.
  const double volume_tolerance(mesh.num_elements() * 8.0 *
                                std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(mesh_volume, box_volume, volume_tolerance)
      << "The mesh's volume does not equal the box's volume.";
  if (abs(mesh_volume - box_volume) > volume_tolerance) {
    return false;
  }

  // C. The mesh conforms to the box's medial axis.
  // C1. No tetrahedron has all four vertices on the box's boundary, i.e.,
  //     each tetrahedron has at least one interior vertex.
  std::vector<int> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    bool tetrahedron_has_an_interior_vertex = false;
    for (int i = 0;
         i < mesh.kVertexPerElement && !tetrahedron_has_an_interior_vertex;
         ++i) {
      tetrahedron_has_an_interior_vertex =
          boundary_vertices.end() == find(boundary_vertices.begin(),
                                          boundary_vertices.end(),
                                          tetrahedron.vertex(i));
    }
    EXPECT_TRUE(tetrahedron_has_an_interior_vertex)
        << "A tetrahedron has all its vertices on the box's boundary.";
    if (!tetrahedron_has_an_interior_vertex) {
      return false;
    }
  }
  // C2. No tetrahedron has all four vertices at the same distance to the
  //     box's boundary.
  // Assume the mesh already passes B2 (no mesh's vertex is outside the box).
  auto distance_to_boundary = [&half_size](const Vector3d& point_in_box) {
    Vector3d abs_point = point_in_box.array().abs();
    // Verify that the point is in the box.
    DRAKE_DEMAND((half_size.array() >= abs_point.array()).all());
    return (half_size - abs_point).minCoeff();
  };
  const double distance_tolerance =
      DistanceToPointRelativeTolerance(half_size.maxCoeff());
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    const double distance_v0 =
        distance_to_boundary(mesh.vertex(tetrahedron.vertex(0)));
    bool different_distance_from_v0 = false;
    for (int i = 1; i < mesh.kVertexPerElement && !different_distance_from_v0;
         ++i) {
      different_distance_from_v0 =
          distance_tolerance <
          abs(distance_v0 -
              distance_to_boundary(mesh.vertex(tetrahedron.vertex(i))));
    }
    EXPECT_TRUE(different_distance_from_v0)
        << "A tetrahedron has all vertices at the same distances to"
           " the box's boundary.";
    if (!different_distance_from_v0) {
      return false;
    }
  }
  // C3. Each tetrahedron conforms to MA.
  // Assume the mesh already passes B2 (no mesh's vertex is outside the box).
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    bool tetrahedron_conform_to_MA =
        IsTetrahedronRespectingMa(tetrahedron, mesh, box, distance_tolerance);
    EXPECT_TRUE(tetrahedron_conform_to_MA)
        << "A tetrahedron does not conform to the medial axis of the box.";
    if (!tetrahedron_conform_to_MA) {
      return false;
    }
  }

  return true;
}

// This test should cover every program statement in MakeBoxVolumeMeshWithMa()
// and its subroutine SplitToTetrahedra(). You can use kcov to verify.
GTEST_TEST(MakeBoxVolumeMeshWithMaTest, StatementCoverage) {
  // A cube allows degeneracy of MA in all three directions, so we can
  // exercise every line of code. Testing with a non-cube box (for
  // example, a box of size 2x4x6) will miss some lines of code.
  const Box box(Box::MakeCube(1.3));
  const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
  EXPECT_EQ(mesh.num_elements(), 12);
  EXPECT_EQ(mesh.num_vertices(), 9);
  EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
}

// This test should cover all boolean conditions of MakeBoxVolumeMeshWithMa().
// We want to test all boolean terms (also known as conditions) in a
// statement like this:
//
//     if (!duplicate_in_i && !duplicate_in_j && !duplicate_in_k) {
//        ...
//     }
//
// We can use this table (T=true, F=false):
// +---------- -- -+---------- -- -+---------------+-----------------+
// |duplicate_in_i |duplicate_in_j |duplicate_in_k | test parameter  |
// +---------- -- -+---------- -- -+---------------+-----------------+
// |       T       |       T       |       T       | Box(2, 2, 2)    |
// |       F       |       F       |       T       | Box(6, 4, 2)    |
// |       T       |       F       |       F       | Box(2, 6, 4)    |
// +---------- -- -+---------- -- -+---------------+-----------------+
//
// We do not test all 2³ = 8 combinations. We only pick enough test parameters
// to cover each boolean term with the values of true and false at least once.
// Notice that the two combinations {(F,F,F),(T,T,T)} would have done the
// job; however, (F,F,F) is infeasible because at least in one of the index
// i,j, or k, we must have duplicated vertices by design.
GTEST_TEST(MakeBoxVolumeMeshWithMaTest, ConditionCoverage) {
  // +---------- -- -+---------- -- -+---------------+-----------------+
  // |duplicate_in_i |duplicate_in_j |duplicate_in_k | test parameter  |
  // +---------- -- -+---------- -- -+---------------+-----------------+
  // |       T       |       T       |       T       | Box(2, 2, 2)    |
  // +---------- -- -+---------- -- -+---------------+-----------------+
  {
    const Box box(2., 2., 2.);
    const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
    EXPECT_EQ(mesh.num_elements(), 12);
    EXPECT_EQ(mesh.num_vertices(), 9);
    EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
  }
  // +---------- -- -+---------- -- -+---------------+-----------------+
  // |duplicate_in_i |duplicate_in_j |duplicate_in_k | test parameter  |
  // +---------- -- -+---------- -- -+---------------+-----------------+
  // |       F       |       F       |       T       | Box(6, 4, 2)    |
  // +---------- -- -+---------- -- -+---------------+-----------------+
  {
    const Box box(6., 4., 2.);
    const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
    EXPECT_EQ(mesh.num_elements(), 24);
    EXPECT_EQ(mesh.num_vertices(), 12);
    EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
  }
  // +---------- -- -+---------- -- -+---------------+-----------------+
  // |duplicate_in_i |duplicate_in_j |duplicate_in_k | test parameter  |
  // +---------- -- -+---------- -- -+---------------+-----------------+
  // |       T       |       F       |       F       | Box(2, 6, 4)    |
  // +---------- -- -+---------- -- -+---------------+-----------------+
  {
    const Box box(2., 6., 4.);
    const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
    EXPECT_EQ(mesh.num_elements(), 24);
    EXPECT_EQ(mesh.num_vertices(), 12);
    EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
  }
}

// This test should cover all representative parameters, i.e., all equivalent
// classes of input boxes for MakeBoxVolumeMeshWithMa(). Here we define the
// equivalent classes of Box(a,b,c) by the repeated box dimensions and their
// relative length to the remaining dimension like this:
//
// a = b = c  a cube
// a = b < c  a long box with two small square faces
// a = b > c  a shallow box with two big square faces
// a < b < c  a box with three distinct dimensions
//
// We do not distinguish a permutation of a,b,c from another, i.e., Box(2,4,6)
// and Box(6,4,2) are considered in the same equivalent class.
GTEST_TEST(MakeBoxVolumeMeshWithMaTest, ParameterValueCoverage) {
  // A box with same three dimensions.
  {
    const Box box(2., 2., 2.);
    const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
    EXPECT_EQ(mesh.num_elements(), 12);
    EXPECT_EQ(mesh.num_vertices(), 9);
    EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
  }
  // A box with two same short dimensions.
  {
    const Box box(2., 4., 2.);
    const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
    EXPECT_EQ(mesh.num_elements(), 16);
    EXPECT_EQ(mesh.num_vertices(), 10);
    EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
  }
  // A box with two same long dimensions.
  {
    const Box box(4., 4., 2.);
    const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
    EXPECT_EQ(mesh.num_elements(), 24);
    EXPECT_EQ(mesh.num_vertices(), 12);
    EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
  }
  // A box with three distinct dimensions.
  {
    const Box box(2., 4., 6.);
    const VolumeMesh<double> mesh = MakeBoxVolumeMeshWithMa<double>(box);
    EXPECT_EQ(mesh.num_elements(), 24);
    EXPECT_EQ(mesh.num_vertices(), 12);
    EXPECT_TRUE(VerifyBoxMeshWithMa(mesh, box));
  }
}

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
        Vector3<double> r_MV = vertices[sequential_index];
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
        box_mesh.CalcTetrahedronVolume(e);
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
  TriangleSurfaceMesh<double> surface_mesh =
      MakeBoxSurfaceMesh<double>(box, target_edge_length);

  const int expect_num_vertices = 114;
  EXPECT_EQ(expect_num_vertices, surface_mesh.num_vertices());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
