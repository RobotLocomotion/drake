#include "drake/geometry/proximity/mesh_intersection.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace internal {

template<typename T>
class SurfaceVolumeIntersectorTester {
 public:
  Vector3<T> CalcIntersection(const Vector3<T>& p_FA, const Vector3<T>& p_FB,
                              const PosedHalfSpace<T>& H_F) {
    return intersect_.CalcIntersection(p_FA, p_FB, H_F);
  }
  void ClipPolygonByHalfSpace(const std::vector<Vector3<T>>& polygon_vertices_F,
                              const PosedHalfSpace<T>& H_F,
                              std::vector<Vector3<T>>* output_vertices_F) {
    intersect_.ClipPolygonByHalfSpace(polygon_vertices_F, H_F,
                                      output_vertices_F);
  }
  void RemoveDuplicateVertices(std::vector<Vector3<T>>* polygon) {
    intersect_.RemoveDuplicateVertices(polygon);
  }
  const std::vector<Vector3<T>>& ClipTriangleByTetrahedron(
      VolumeElementIndex element, const VolumeMesh<T>& volume_M,
      SurfaceFaceIndex face, const SurfaceMesh<T>& surface_N,
      const math::RigidTransform<T>& X_MN) {
    return intersect_.ClipTriangleByTetrahedron(element, volume_M, face,
                                                surface_N, X_MN);
  }
  bool IsFaceNormalAlongPressureGradient(
      const VolumeMeshField<T, T>& volume_field_M,
      const SurfaceMesh<T>& surface_N, const math::RigidTransform<T>& X_MN,
      const VolumeElementIndex& tet_index, const SurfaceFaceIndex& tri_index) {
    return intersect_.IsFaceNormalAlongPressureGradient(
        volume_field_M, surface_N, X_MN, tet_index, tri_index);
  }

 private:
  SurfaceVolumeIntersector<T> intersect_;
};

namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::unique_ptr;

// TODO(SeanCurtis-TRI): Robustly confirm that epsilon of 1e-14 is correct for
//  determining that the intersection is valid. One would suppose that it will
//  depend on the magnitude of the values in play.

// TODO(DamrongGuoy): More comprehensive tests.
GTEST_TEST(MeshIntersectionTest, CalcIntersection) {
  const double kEps = std::numeric_limits<double>::epsilon();
  // TODO(SeanCurtis-TRI): This test has too many zeros in it (the normal is
  //  [1, 0, 0] -- that is not a robust test. Pick a more arbitrary normal.
  // Half space {(x,y,z) : x <= 2.0}
  const Vector3d unit_normal_H = Vector3d::UnitX();
  const double plane_offset = 2.0;
  const PosedHalfSpace<double> half_space_H(unit_normal_H,
                                            plane_offset * unit_normal_H);

  // The line AB intersects the plane of the half space.
  {
    const Vector3d p_HA = Vector3d::Zero();
    const Vector3d p_HB(4, 6, 10);
    const Vector3d intersection =
        SurfaceVolumeIntersectorTester<double>().CalcIntersection(
            p_HA, p_HB, half_space_H);
    const Vector3d expect_intersection(2, 3, 5);
    EXPECT_LE((expect_intersection - intersection).norm(), kEps);
  }

  // The line AB is almost parallel to the plane of the half space.
  {
    const Vector3d p_HA(plane_offset + 2.0 * kEps, 0., 0.);
    const Vector3d p_HB(plane_offset - 2.0 * kEps, 1., 1.);
    const Vector3d intersection =
        SurfaceVolumeIntersectorTester<double>().CalcIntersection(
            p_HA, p_HB, half_space_H);
    const Vector3d expect_intersection(2., 0.5, 0.5);
    EXPECT_LE((expect_intersection - intersection).norm(), kEps);
  }

  // TODO(SeanCurtis-TRI): Confirm death test in debug mode for if the points
  //  don't properly "straddle" the boundary plane.
  //  - Both on negative, both on positive, both *on* the plane.
  //  - parallel negative, parallel positive, parallel on the plane.
}

// TODO(DamrongGuoy): Move the definition of this function here after 11612
//  landed. The definition is currently down below here.
template<typename T>
bool CompareConvexPolygon(const std::vector<Vector3<T>>& polygon0,
                          const std::vector<Vector3<T>>& polygon1);

// Although polygons with zero, one, or two vertices are valid input and are
// treated correctly by this function, they are not tested as being a
// meaningless operation.
GTEST_TEST(MeshIntersectionTest, ClipPolygonByHalfSpace) {
  // All quantities are (measured and) expressed in the half space frame H.

  // TODO(SeanCurtis-TRI): This half space does *not* tax the numerics at all.
  //  The normal is [1, 0, 0] which kills most of the multiplication. Pick a
  //  more arbitrarily oriented normal and an offset that is not perfectly
  //  represented as a power of two.
  // Half space {(x,y,z) : x <= 2.0}
  const Vector3d unit_normal_H = Vector3d::UnitX();
  const double offset = 2.0;
  const PosedHalfSpace<double> half_space_H(unit_normal_H,
                                            offset * unit_normal_H);

  // The input polygon is half inside the half space and half outside the
  // half space. Expect the output polygon to be half of the input polygon.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {1., 0., 0.},
        {1., 2., 0.},
        {3., 2., 0.},
        {3., 0., 0.}
    };
    const std::vector<Vector3d> expect_output_polygon{
        {1., 0., 0.},
        {1., 2., 0.},
        {2., 2., 0.},
        {2., 0., 0.},
    };
    // clang-format on
    // NOTE: By construction, we know the expected output polygon is planar
    // (i.e., z = 0 for all vertices). There is no need to test this explicitly.
    // Also, by construction, the winding matches, so we will also not be
    // explicitly testing that.
    std::vector<Vector3d> output_polygon;
    SurfaceVolumeIntersectorTester<double>().ClipPolygonByHalfSpace(
        input_polygon, half_space_H, &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_output_polygon, output_polygon));
  }
  // The input polygon is on the plane X=0, which is parallel to the plane of
  // the half space and is completely inside the half space. Expect the input
  // polygon and the output polygon to be the same.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {0., 0., 0.},
        {0., 1., 0.},
        {0., 1., 1.},
        {0., 0., 1.}
    };
    // clang-format on
    // Because we expect the output polygon to *be* the input polygon, we don't
    // need to explicitly test planarity or winding.
    std::vector<Vector3d> output_polygon;
    SurfaceVolumeIntersectorTester<double>().ClipPolygonByHalfSpace(
        input_polygon, half_space_H, &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(input_polygon, output_polygon));
  }
  // The input polygon is on the plane X=3, which is parallel to the plane of
  // the half space and is completely outside the half space. Expect the output
  // polygon to be empty.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {3., 0., 0.},
        {3., 1., 0.},
        {3., 1., 1.},
        {3., 0., 1.}
    };
    // clang-format on
    // Empty polygons have no winding and no planarity.
    std::vector<Vector3d> output_polygon;
    SurfaceVolumeIntersectorTester<double>().ClipPolygonByHalfSpace(
        input_polygon, half_space_H, &output_polygon);
    const std::vector<Vector3d> empty_polygon;
    EXPECT_TRUE(CompareConvexPolygon(empty_polygon, output_polygon));
  }
  // The input polygon is on the plane X=2 of the half space. Expect the input
  // polygon and the output polygon to be the same.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {2., 0., 0.},
        {2., 1., 0.},
        {2., 1., 1.},
        {2., 0., 1.}
    };
    // clang-format on
    // Because we expect the output polygon to *be* the input polygon, we don't
    // need to explicitly test planarity or winding.
    std::vector<Vector3d> output_polygon;
    SurfaceVolumeIntersectorTester<double>().ClipPolygonByHalfSpace(
        input_polygon, half_space_H, &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(input_polygon, output_polygon));
  }
  // The input polygon is outside the half space, but it has one edge on the
  // plane of the half space. Expect the output polygon to be a zero-area
  // rectangle with two pairs of duplicated vertices.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {2., 0., 0.},
        {2., 2., 0.},
        {3., 2., 0.},
        {3., 0., 0.}
    };
    const std::vector<Vector3d> expect_output_polygon{
        {2., 0., 0.},
        {2., 0., 0.},
        {2., 2., 0.},
        {2., 2., 0.},
    };
    // clang-format on
    // By construction, expected output is planar (z = 0 for all vertices). It
    // has no area, so winding is immaterial.
    std::vector<Vector3d> output_polygon;
    SurfaceVolumeIntersectorTester<double>().ClipPolygonByHalfSpace(
        input_polygon, half_space_H, &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_output_polygon, output_polygon));
  }
  // The input polygon is outside the half space, but it has one vertex on the
  // plane of the half space. Expect the output polygon to be a zero-area
  // triangle with three duplicated vertices.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {2., 0., 0.},
        {3., 2., 0.},
        {3., 0., 0.}
    };
    const std::vector<Vector3d> expect_output_polygon{
        {2., 0., 0.},
        {2., 0., 0.},
        {2., 0., 0.}
    };
    // clang-format on
    // By construction, expected output is planar (z = 0 for all vertices). It
    // has no area, so winding is immaterial.
    std::vector<Vector3d> output_polygon;
    SurfaceVolumeIntersectorTester<double>().ClipPolygonByHalfSpace(
        input_polygon, half_space_H, &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_output_polygon, output_polygon));
  }
  // TODO(SeanCurtis-TRI): Clip a triangle into a quad. Clip a triangle into a
  // triangle.
}

GTEST_TEST(MeshIntersectionTest, RemoveDuplicateVertices) {
  // ABCD: No duplicate vertices. Expect no change to the polygon.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {0., 0., 0.},
        {0., 1., 0.},
        {0., 1., 1.},
        {0., 0., 1.}
    };
    // clang-format on
    std::vector<Vector3d> output_polygon = input_polygon;
    SurfaceVolumeIntersectorTester<double>().RemoveDuplicateVertices(
        &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(input_polygon, output_polygon));
  }
  // AAA: Three identical vertices reduced to a single vertex A.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {2., 0., 0.},
        {2., 0., 0.},
        {2., 0., 0.}
    };
    const std::vector<Vector3d> expect_single_vertex{
        {2., 0., 0.}
    };
    // clang-format on
    std::vector<Vector3d> output_polygon = input_polygon;
    SurfaceVolumeIntersectorTester<double>().RemoveDuplicateVertices(
        &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_single_vertex, output_polygon));
  }
  // AABB: Two pairs of duplicate vertices. Reduced to two vertices AB.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {2., 0., 0.},
        {2., 0., 0.},
        {2., 2., 0.},
        {2., 2., 0.},
    };
    const std::vector<Vector3d> expect_two_vertices{
        {2., 0., 0.},
        {2., 2., 0.}
    };
    // clang-format on
    std::vector<Vector3d> output_polygon = input_polygon;
    SurfaceVolumeIntersectorTester<double>().RemoveDuplicateVertices(
        &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_two_vertices, output_polygon));
  }
  // TODO(SeanCurtis-TRI): Add tests:
  //  1. Test two pairs of duplicates: ABBA.
  //  2. ABB'C where |B - B'| = 2 * epsilon.
  //  3. Change epsilon to the largest value that still fails. Repeat one
  //     of the epsilon tests with a value just *larger* than the "coincident
  //     point" threshold and confirm that it doesn't collapse.

  // ABCA' (where |A - A'| = 2 * epsilon. The first and the last vertex are
  // identical within 2 * epsilon. Expect ABC.
  {
    // clang-format off
    const std::vector<Vector3d> input_polygon{
        {2., 0., 0.},
        {2., 0., 1.},
        {2., 1., 0.},
        {2., 0., 2. * std::numeric_limits<double>::epsilon()},
    };
    const std::vector<Vector3d> expect_three_vertices{
        {2., 0., 0.},
        {2., 0., 1.},
        {2., 1., 0.},
    };
    // clang-format on
    std::vector<Vector3d> output_polygon = input_polygon;
    SurfaceVolumeIntersectorTester<double>().RemoveDuplicateVertices(
        &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_three_vertices, output_polygon));
  }
}

// TODO(SeanCurtis-TRI): Too many zeros; it reduces the power of the test
//  because so much of the math simply disappears. Reframe these tests with
//  non-trivial triangles and tets -- the recommendation is to formulate these
//  in an easy frame, find the answer in the easy frame, and then transform
//  them all into an obnoxious frame and solve it there. This applies to both
//  triangle and tetrahedron.

// Generates a trivial surface mesh consisting of one triangle with vertices
// at the origin and on the X- and Y-axes. We will use it for testing
// triangle-tetrahedron intersection. By design, face(0, 1, 2) lies on the
// same plane as the shared face of the trivial volume mesh. (When X_VS = I, for
// V the frame of the volume mesh and S the frame of the surface mesh.)
//
//      +Z
//       |
//       |
//       |
//       |
//     v0+------v2---+Y
//      /
//     /
//   v1
//   /
// +X
//
template<typename T>
unique_ptr<SurfaceMesh<T>> TrivialSurfaceMesh() {
  const int face_data[3] = {0, 1, 2};
  std::vector<SurfaceFace> faces{SurfaceFace(face_data)};
  const Vector3<T> vertex_data[3] = {
      Vector3<T>::Zero(),
      Vector3<T>::UnitX(),
      Vector3<T>::UnitY()
  };
  std::vector<SurfaceVertex<T>> vertices;
  for (auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return std::make_unique<SurfaceMesh<T>>(std::move(faces),
                                          std::move(vertices));
}

// Generates a trivial volume mesh consisting of two tetrahedrons with
// vertices on the coordinate axes and the origin like this:
//
//      +Z
//       |
//       v3
//       |
//       |
//     v0+------v2---+Y
//      /|
//     / |
//   v1  v4
//   /   |
// +X    |
//      -Z
//
template <typename T>
unique_ptr<VolumeMesh<T>> TrivialVolumeMesh(
    const math::RigidTransform<T>& X_MN = math::RigidTransform<T>::Identity()) {
  const int element_data[2][4] = {
      {0, 1, 2, 3},
      {0, 2, 1, 4}};
  std::vector<VolumeElement> elements;
  for (const auto& element : element_data) {
    elements.emplace_back(element);
  }
  const Vector3<T> vertex_data[5] = {
      X_MN * Vector3<T>::Zero(),
      X_MN * Vector3<T>::UnitX(),
      X_MN * Vector3<T>::UnitY(),
      X_MN * Vector3<T>::UnitZ(),
      X_MN * (-Vector3<T>::UnitZ())
  };
  std::vector<VolumeVertex<T>> vertices;
  for (auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return std::make_unique<VolumeMesh<T>>(std::move(elements),
                                         std::move(vertices));
}

template<typename T>
unique_ptr<VolumeMeshFieldLinear<T, T>> TrivialVolumeMeshField(
    const VolumeMesh<T>* volume_mesh) {
  // Pressure field value pᵢ at vertex vᵢ.
  const T p0{0.};
  const T p1{0.};
  const T p2{0.};
  const T p3{1e+7};
  const T p4{1e+10};
  std::vector<T> p_values = {p0, p1, p2, p3, p4};
  DRAKE_DEMAND(5 == volume_mesh->num_vertices());
  auto volume_mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "pressure", std::move(p_values), volume_mesh);

  return volume_mesh_field;
}

// Checks whether two convex polygons are equal. Here, equality means they
// have the same set of vertices in the same cyclic order. Any corresponding
// pair of vertices between the two polygons do not necessarily have the same
// vertex indices.
//     We use this function in testing ClipTriangleByTetrahedron. Expect to
// work for polygons within 10 meters from the origin since we use absolute
// tolerance. Do not use this function in production.
template<typename T>
bool CompareConvexPolygon(const std::vector<Vector3<T>>& polygon0,
                          const std::vector<Vector3<T>>& polygon1) {
  const int polygon0_size = polygon0.size();
  const int polygon1_size = polygon1.size();
  // Check that they have the same number of vertices.
  if (polygon0_size != polygon1_size) return false;
  // Two null polygons are considered equal.
  if (polygon0_size == 0) return true;
  // Two vertices are the same if they are within a very small tolerance from
  // the other.
  auto are_same = [](const Vector3<T>& u, const Vector3<T>& v) -> bool {
    DRAKE_DEMAND(u.norm() < T(10.0));
    DRAKE_DEMAND(v.norm() < T(10.0));
    // TODO(SeanCurtis-TRI): Ideally, this should be expressed in terms of the
    //  epsilon used in `mesh_intersection.h` and not redefine/rejustify a
    //  particular value.
    // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
    // too small.
    const T kEps(1e-14);
    return (u - v).norm() < kEps;
  };

  // Find the first vertex in polygon1 that matches vertex 0 of polygon0.
  // This is the unary predicate for finding the matching vertex in polygon1.
  auto matches = [&are_same](const Vector3<T>& u) {
    return [&are_same, &u](const Vector3<T>& v) { return are_same(u, v); };
  };
  auto it = std::find_if(polygon1.begin(), polygon1.end(),
      matches(polygon0[0]));
  if (it == polygon1.end()) {
    return false;
  }

  // Vertex i1 of polygon1 matches vertex 0 of polygon0.
  int i1 = it - polygon1.begin();
  // Vertex 0 was checked already. Go to the next one.
  for (int i0 = 1; i0 < polygon0_size; ++i0) {
    i1 = (i1 + 1) % polygon1_size;
    if (!are_same(polygon0[i0], polygon1[i1])) return false;
  }
  return true;
}

GTEST_TEST(MeshIntersectionTest, ClipTriangleByTetrahedron) {
  auto volume_M = TrivialVolumeMesh<double>();
  auto surface_N = TrivialSurfaceMesh<double>();
  // TODO(SeanCurtis-TRI): Seeing these two types together suggests there's a
  //  naming problem. Volume *element* and surface *face*. Why is it not
  //  surface element? Element doesn't imply 3D. Alternatively, I favor
  //  TetrahedronIndex and TriangleIndex (or TetIndex and TriIndex,
  //  respectively) as being far more literal. It's not like each of those
  //  surface types are ever designed to support any other kind of "element".
  const VolumeElementIndex element0(0);
  const VolumeElementIndex element1(1);
  SurfaceFaceIndex face(0);
  const std::vector<Vector3d> empty_polygon;

  // The triangle is outside the tetrahedron `element0` with one vertex on a
  // face of the tetrahedron. Expect the output polygon to be empty.
  {
    const auto X_MN = RigidTransformd(Vector3d::UnitX());
    const std::vector<Vector3d> polygon =
        SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
            element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3d> expect_empty_polygon;
    EXPECT_TRUE(CompareConvexPolygon(expect_empty_polygon, polygon));
  }

  // The triangle is outside the tetrahedron `element0` with one edge on a
  // face of the tetrahedron. Expect the output polygon to be empty.
  {
    const auto X_MN = RigidTransformd(RollPitchYawd(0, 0, M_PI_2),
                                             Vector3d::Zero());
    const std::vector<Vector3d> polygon =
        SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
            element0, *volume_M, face, *surface_N, X_MN);
    EXPECT_TRUE(CompareConvexPolygon(empty_polygon, polygon));
  }

  // The triangle coincides with the shared face between the two tetrahedra.
  // Expect "double count". Both tetrahedral elements give the same intersecting
  // polygon, which is the triangle.
  // TODO(DamrongGuoy): Change the expectation when we solve the "double
  //  count" problem.
  {
    const auto X_MN = RigidTransformd::Identity();
    const std::vector<Vector3d> polygon0_M =
        SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
            element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3d> polygon1_M =
        SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
            element1, *volume_M, face, *surface_N, X_MN);
    // clang-format off
    const std::vector<Vector3d> expect_triangle_M{
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0}};
    // clang-format on
    EXPECT_TRUE(CompareConvexPolygon(expect_triangle_M, polygon0_M));
    EXPECT_TRUE(CompareConvexPolygon(expect_triangle_M, polygon1_M));
  }

  // The triangle intersects tetrahedron `element0` and is clipped to a smaller
  // triangle.
  {
    const auto X_MN = RigidTransformd(Vector3d(0, 0, 0.5));
    const std::vector<Vector3d> polygon0_M =
        SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
            element0, *volume_M, face, *surface_N, X_MN);
    // clang-format off
    const std::vector<Vector3d> expect_triangle_M{
        {0,   0,   0.5},
        {0.5, 0,   0.5},
        {0,   0.5, 0.5}};
    // clang-format on
    EXPECT_TRUE(CompareConvexPolygon(expect_triangle_M, polygon0_M));
  }

  // The triangle lies completely outside tetrahedron `element1` and the result
  // is the empty polygon.
  {
    const auto X_MN = RigidTransformd(Vector3d(0, 0, 0.5));
    const std::vector<Vector3d> polygon1_M =
        SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
            element1, *volume_M, face, *surface_N, X_MN);
    EXPECT_TRUE(CompareConvexPolygon(empty_polygon, polygon1_M));
  }

  // The triangle intersects the tetrahedron `element0` such that the result is
  // a quad.
  {
    const auto X_MN = RigidTransformd(RollPitchYawd(0, 0, M_PI),
                                             Vector3d(0.5, 0.5, 0));
    const std::vector<Vector3d> polygon0_M =
        SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
            element0, *volume_M, face, *surface_N, X_MN);
    // clang-format off
    const std::vector<Vector3d> expect_square_M{
        {0,   0,   0},
        {0.5, 0,   0},
        {0.5, 0.5, 0},
        {0,   0.5, 0}};
    // clang-format on
    EXPECT_TRUE(CompareConvexPolygon(expect_square_M, polygon0_M));
  }
  // TODO(DamrongGuoy): Test other cases like:
  //  - the intersecting polygon is a pentagon,
  //  - the intersecting polygon is a hexagon,
  //  - More general X_MN.
}

// Tests a triangle intersect a tetrahedron into a heptagon (seven-sided
// polygon). Strategy:
// 1. Create a tetrahedron that intersects the X-Y plane (Z=0) into a square.
//    A tetrahedron with vertices v0,v1,v2,v3 in this picture will do:
//
//                   +Z   ● v1            v0: (2, 0, 2)
//                    |  /|               v1: (-2, 0, 2)
//                    | / |               v2: (0, 2, -2)
//                    |/  +               v3: (0, -2, -2)
//                    +  /
//                   /| /
//                  / |/
//           +-----/--+------+----+Y
//           | v0 ●  /|      |
//           |    | / |      |
//        v3 ●----|/--+------⚫ v2
//                +
//               /
//              /
//            +X
//
// This tetrahedron intersects the X-Y plane into a square with vertices at
// u0 = (v0+v2)/2, u1 = (v1+v2)/2, u2 = (v1+v3)/2, u3 = (v0+v3)/2,
// u0 = (1,1,0),   u1 = (-1,1,0),  u2 = (-1,-1,0), u3 = (1,-1,0).
//
// In X-Y plane, the square u0, u1, u2, u3 (●) will look like this:
//
//
//               +Y          ◯ t0
//                ┆
//     u1 ●━━━━━━━1━━━━━━━● u0
//        ┃       ┆       ┃
//        ┃       ┆       ┃
//        ┃       ┆       ┃
// t1 ◯┄┄┄╂┄┄┄┄┄┄┄┼┄┄┄┄┄┄┄1┄┄┄ +X
//        ┃       ┆       ┃
//        ┃       ┆       ┃
//        ┃       ┆       ┃
//     u2 ●━━━━━━━┿━━━━━━━● u3
//                ┆
//                ◯ t2
//
// 2. Create a triangle on the X-Y plane that intersects the square into a
//    heptagon. A triangle with vertices (◯) t0(1.5,1.5,0), t1(-1.5,0,0), and
//    t2(0,-1.5,0) will do. See the above picture.
//
GTEST_TEST(MeshIntersectionTest, ClipTriangleByTetrahedronIntoHeptagon) {
  unique_ptr<VolumeMesh<double>> volume_M;
  {
    const int element_data[4] = {0, 1, 2, 3};
    std::vector<VolumeElement> elements{VolumeElement(element_data)};
    // clang-format off
    const Vector3d vertex_data[4] = {
        { 2,  0,  2},
        {-2,  0,  2},
        { 0,  2, -2},
        { 0, -2, -2}
    };
    // clang-format on
    std::vector<VolumeVertex<double>> vertices;
    for (auto& vertex : vertex_data) {
      vertices.emplace_back(vertex);
    }
    volume_M = std::make_unique<VolumeMesh<double>>(std::move(elements),
                                                    std::move(vertices));
  }
  unique_ptr<SurfaceMesh<double>> surface_N;
  {
    const int face_data[3] = {0, 1, 2};
    std::vector<SurfaceFace> faces{SurfaceFace(face_data)};
    // clang-format off
    const Vector3d vertex_data[3] = {
        {1.5,   1.5, 0.},
        {-1.5,  0.,  0.},
        {0.,   -1.5, 0.}};
    // clang-format on
    std::vector<SurfaceVertex<double>> vertices;
    for (auto& vertex : vertex_data) {
      vertices.emplace_back(vertex);
    }
    surface_N = std::make_unique<SurfaceMesh<double>>(std::move(faces),
                                                      std::move(vertices));
  }
  const VolumeElementIndex tetrahedron(0);
  const SurfaceFaceIndex triangle(0);
  const auto X_MN = RigidTransformd::Identity();
  const std::vector<Vector3d> polygon_M =
      SurfaceVolumeIntersectorTester<double>().ClipTriangleByTetrahedron(
          tetrahedron, *volume_M, triangle, *surface_N, X_MN);
  // clang-format off
  const std::vector<Vector3d> expect_heptagon_M{
      {1.,    1.,   0.},
      {0.5,   1.,   0.},
      {-1.,   0.25, 0.},
      {-1.,  -0.5,  0.},
      {-0.5, -1.,   0.},
      {0.25, -1.,   0.},
      {1.,    0.5,  0.}};
  // clang-format on
  EXPECT_EQ(7, polygon_M.size());
  EXPECT_TRUE(CompareConvexPolygon(expect_heptagon_M, polygon_M));
}

GTEST_TEST(MeshIntersectionTest, IsFaceNormalAlongPressureGradient) {
  // It is ok to use the trivial mesh and trivial mesh field in this test.
  // The function under test asks for the gradient values and operates on it.
  // It is not responsible for making sure that the gradient is computed
  // correctly -- that is tested elsewhere.

  // Let F be the expressed-in frame of the trivial volume mesh. In frame F,
  // the tetrahedron Element_0 is above the X-Y plane, and its trivial volume
  // mesh field has the gradient vector in Element_0 in +Z direction of frame
  // F. We will use the following general rigid transform X_MF to express the
  // volume mesh and its field in another frame M, so the test is more general.
  RigidTransformd X_MF(RollPitchYawd(M_PI_4, 2. * M_PI / 3., M_PI / 6.),
                       Vector3d(1.1, 2.5, 4.0));
  const auto volume_M = TrivialVolumeMesh<double>(X_MF);
  const auto volume_field_M = TrivialVolumeMeshField<double>(volume_M.get());
  // Rigid surface mesh N has the triangle Face_0 with its face normal vector
  // in +Z direction of N's frame.
  const auto rigid_N = TrivialSurfaceMesh<double>();

  // We will set the pose of SurfaceMesh N in frame M so that triangle
  // Face_0 of N has its face normal vector make various angles with the
  // gradient vector in tetrahedron Element_0.
  struct TestData {
    double angle;        // Angle between the face normal and the gradient.
    bool expect_result;  // true when `angle` <  hard-coded threshold 5π/8.
  } test_data[]{{0, true},
                 {M_PI_2, true},
                 {(5. * M_PI / 8.) * 0.99, true},   // slightly less than 5π/8
                 {(5. * M_PI / 8.) * 1.01, false},  // slightly more than 5π/8
                 {3. * M_PI_4, false},
                 {M_PI, false}};

  // Whether triangle Face_0 intersects tetrahedron Element_0 is not
  // relevant to this test because IsFaceNormalAlongPressureGradient()
  // only checks the angle between the normal and the gradient without
  // triangle-tetrahedron intersection test.
  for (const TestData& t : test_data) {
    // First we use a simple pose of surface N in frame F of the trivial volume
    // mesh, so we can check the angle threshold conveniently.
    const auto X_FN =
        RigidTransformd(RollPitchYawd(t.angle, 0, 0), Vector3d::Zero());
    // Then, we use the general rigid transform X_MF to change simple X_FN to
    // general X_MN as an argument to the tested function
    // IsFaceNormalAlongPressureGradient().
    const auto X_MN = X_MF * X_FN;
    EXPECT_EQ(t.expect_result,
              SurfaceVolumeIntersectorTester<double>()
                  .IsFaceNormalAlongPressureGradient(
                      *volume_field_M, *rigid_N, X_MN, VolumeElementIndex(0),
                      SurfaceFaceIndex(0)));
  }
}

// Given a triangle in a surface mesh, reports the tet in the volume mesh that
// completely contains the triangle. Throws if a test cannot be identified.
template <typename T>
VolumeElementIndex GetTetForTriangle(const SurfaceMesh<T>& surface_S,
                                     SurfaceFaceIndex f,
                                     const VolumeMesh<T>& volume_V,
                                     const RigidTransform<T>& X_VS) {
  const std::vector<SurfaceVertex<T>>& vertices_S = surface_S.vertices();

  // Each triangle lies completely within one tet in the volume mesh. The
  // gradient value reported should be that of that tet. So, we'll grab
  // the centroid of each triangle, find the tet it lies in, and confirm
  // that the pressure gradient on that triangle matches the tet.
  const SurfaceFace& face = surface_S.element(f);
  const Vector3<T> p_SC =
      (vertices_S[face.vertex(0)].r_MV() + vertices_S[face.vertex(1)].r_MV() +
       vertices_S[face.vertex(2)].r_MV()) /
      3;
  const Vector3<T> p_VC = X_VS * p_SC;
  for (VolumeElementIndex e(0); e < volume_V.num_elements(); ++e) {
    auto bary_e = volume_V.CalcBarycentric(p_VC, e);
    if ((bary_e.array() >= 0).all() && (bary_e.array() <= 1).all()) {
      return e;
    }
  }
  throw std::logic_error(fmt::format(
      "Surface triangle was unable to place triangle {} in any tetrahedron",
      f));
}

GTEST_TEST(MeshIntersectionTest, SampleVolumeFieldOnSurface) {
  auto volume_M = TrivialVolumeMesh<double>();
  const Bvh<VolumeMesh<double>> bvh_volume_M(*volume_M);
  auto volume_field_M = TrivialVolumeMeshField<double>(volume_M.get());
  auto rigid_N = TrivialSurfaceMesh<double>();
  const Bvh<SurfaceMesh<double>> bvh_rigid_N(*rigid_N);
  // Transform the surface (single triangle) so that it intersects with *both*
  // tets in the volume mesh. The surface lies on the y = 0.75 plane.
  // Each tet gets intersected into a isosceles right triangle with a leg
  // length of 0.25m.
  // Construct the 90-degree rotation around the x-axis perfectly so there's
  // no rounding error in our calculations.
  const auto R_MN = RotationMatrixd::MakeFromOrthonormalColumns(
      {1, 0, 0}, {0, 0, 1}, {0, -1, 0});
  const RigidTransformd X_MN(R_MN, Vector3d(-0.1, 0.75, -0.25));
  std::vector<Vector3<double>> grad_eM_M;

  unique_ptr<SurfaceMesh<double>> surface_M;
  unique_ptr<SurfaceMeshFieldLinear<double, double>> e_field;
  SurfaceVolumeIntersector<double>().SampleVolumeFieldOnSurface(
      *volume_field_M, bvh_volume_M, *rigid_N, bvh_rigid_N, X_MN, &surface_M,
      &e_field, &grad_eM_M);

  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_EQ(6, surface_M->num_faces());

  // The geometries M and N intersect such that both tets get sliced into
  // identical right, isosceles triangles (with a leg length of 0.25m). The
  // total area is 2 * 0.25**2 / 2 = 0.25**2.
  const double expect_area = 0.25 * 0.25;
  EXPECT_NEAR(expect_area, surface_M->total_area(), kEps);

  // Here we exploit the simplicity of TrivialVolumeMeshField<>() to check
  // the field value. The test of field evaluation with more complex field
  // values are in the unit test of VolumeMeshFieldLinear<>. We know that
  // mesh vertices on the z = 0 plane must have zero pressure and two vertices
  // at z = +/-0.25m have pressure values of 0.25 * 1e7 and 0.25 * 1e10,
  // respectively. However, for an epsilon deviation, the pressure can
  // vary as much as kEps * max_pressure. So, we'll define a custom threshold
  // for *this* test. Note: we're skipping the vertices located at the
  // triangle centroids.
  const double kEpsPressure = kEps * 1e10;
  const std::vector<SurfaceVertex<double>>& vertices = surface_M->vertices();
  bool domain_checked[] = {false, false, false};
  for (SurfaceVertexIndex v(0); v < surface_M->num_vertices(); ++v) {
    const double p_MV_z = vertices[v].r_MV()[2];
    if (std::abs(p_MV_z) < kEps) {
      ASSERT_NEAR(e_field->EvaluateAtVertex(v), 0.0, kEpsPressure);
      domain_checked[0] = true;
    } else if (std::abs(p_MV_z - 0.25) < kEps) {
      ASSERT_NEAR(e_field->EvaluateAtVertex(v), 0.25 * 1e7, kEpsPressure);
      domain_checked[1] = true;
    } else if (std::abs(p_MV_z + 0.25) < kEps) {
      ASSERT_NEAR(e_field->EvaluateAtVertex(v), 0.25 * 1e10, kEpsPressure);
      domain_checked[2] = true;
    }
  }
  // Confirm the e_field tests didn't pass by omission.
  ASSERT_TRUE(domain_checked[0])
      << "Assumptions have been broken! In testing e_field, no vertex was on "
         "the z = 0 plane.";
  ASSERT_TRUE(domain_checked[1])
      << "Assumptions have been broken! In testing e_field, no vertex was "
         "located at z = 0.25";
  ASSERT_TRUE(domain_checked[2])
      << "Assumptions have been broken! In testing e_field, no vertex was "
         "located at z = -0.25";

  // Test the face normals of resulting mesh. Because the 'trivial' surface
  // mesh is a single triangle, all triangles in the resulting mesh should
  // have the same normal.
  using FIndex = SurfaceFaceIndex;
  ASSERT_TRUE(
      CompareMatrices(rigid_N->face_normal(FIndex{0}), Vector3d::UnitZ()));
  for (FIndex f(0); f < surface_M->num_faces(); ++f) {
    EXPECT_TRUE(CompareMatrices(surface_M->face_normal(f),
                                X_MN.rotation() * Vector3d::UnitZ()));
  }

  // Only the soft volume mesh provides gradients.
  const std::vector<SurfaceFace>& faces = surface_M->faces();
  ASSERT_EQ(faces.size(), grad_eM_M.size());
  for (FIndex f(0); f < surface_M->num_elements(); ++f) {
    const VolumeElementIndex t =
        GetTetForTriangle(*surface_M, f, *volume_M, {});
    ASSERT_TRUE(
        CompareMatrices(grad_eM_M[f], volume_field_M->EvaluateGradient(t)));
  }
  // By design, we wanted to have *different* pressure gradients present
  // in the mesh (hence the reason for intersecting both tetrahedra). Let's
  // confirm that is the case. We assume the first and last triangles in
  // the surface are from *different* tetrahedra. The pressure increases
  // as we move away from the z = 0 plane. So, they'll have different signs
  // and compare as different with a *massive* tolerance (here equal to the
  // maximum pressure value).
  EXPECT_FALSE(CompareMatrices(grad_eM_M.front(), grad_eM_M.back(), 1e10));
}

// Generates a volume mesh of an octahedron comprising of eight tetrahedral
// elements with vertices on the coordinate axes and the origin like this:
//
//                +Z   -X
//                 |   /
//              v5 ●  ● v3
//                 | /
//       v4     v0 |/
//  -Y----●--------●------●----+Y
//                /|      v2
//               / |
//           v1 ●  ● v6
//             /   |
//           +X    |
//                -Z
//
template<typename T>
unique_ptr<VolumeMesh<T>> OctahedronVolume() {
  const int element_data[8][4] = {
      // The top four tetrahedrons share the top vertex v5.
      {0, 1, 2, 5}, {0, 2, 3, 5}, {0, 3, 4, 5}, {0, 4, 1, 5},
      // The bottom four tetrahedrons share the bottom vertex v6.
      {0, 2, 1, 6}, {0, 3, 2, 6}, {0, 4, 3, 6}, {0, 1, 4, 6}
  };
  std::vector<VolumeElement> elements;
  for (const auto& element : element_data) {
    elements.emplace_back(element);
  }
  // clang-format off
  const Vector3<T> vertex_data[7] = {
      { 0,  0,  0},
      { 1,  0,  0},
      { 0,  1,  0},
      {-1,  0,  0},
      { 0, -1,  0},
      { 0,  0,  1},
      { 0,  0, -1}};
  // clang-format on
  std::vector<VolumeVertex<T>> vertices;
  for (const auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return std::make_unique<VolumeMesh<T>>(std::move(elements),
                                         std::move(vertices));
}

template<typename T>
unique_ptr<VolumeMeshFieldLinear<T, T>> OctahedronPressureField(
    VolumeMesh<T>* volume_mesh) {
  // The field is 0 on the boundary and linearly increasing to 1e7 at the
  // center of the octahedron.
  std::vector<T> values{1e7, 0, 0, 0, 0, 0, 0};
  return std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "pressure", std::move(values), volume_mesh);
}

// Generates a simple surface mesh of a pyramid with vertices on the
// coordinate axes and the origin like this:
//
//                +Z   -X
//                 |   /
//              v5 ●  ● v3
//                 | /
//        v4    v0 |/
//  -Y-----●-------●------●---+Y
//                /      v2
//               /
//              ● v1
//             /
//           +X
//
template<typename T>
unique_ptr<SurfaceMesh<T>> PyramidSurface() {
  const int face_data[8][3] = {
      // The top four faces share the apex vertex v5.
      {1, 2, 5},
      {2, 3, 5},
      {3, 4, 5},
      {4, 1, 5},
      // The bottom four faces share the origin v0.
      {4, 3, 0},
      {3, 2, 0},
      {2, 1, 0},
      {1, 4, 0}
  };
  std::vector<SurfaceFace> faces;
  for (auto& face : face_data) {
    faces.emplace_back(face);
  }
  // clang-format off
  const Vector3<T> vertex_data[6] = {
      { 0,  0, 0},
      { 1,  0, 0},
      { 0,  1, 0},
      {-1,  0, 0},
      { 0, -1, 0},
      { 0,  0, 1}
  };
  // clang-format on
  std::vector<SurfaceVertex<T>> vertices;
  for (auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return std::make_unique<SurfaceMesh<T>>(std::move(faces),
                                          std::move(vertices));
}

// Tests the generation of the ContactSurface between a soft volume and rigid
// surface. This highest-level function's primary responsibility is to make
// sure that the resulting ContactSurface satisfies the invariant id_M < id_N.
// To that end, it computes the contact surface twice, with the ids reversed
// and confirms the results reflect that: (i.e., vertex positions are different,
// gradients are mirrored.) The difference test is coarsely sampled and assumes
// that some good results are correlated with all good results based on the
// unit tests for ContactSurface.
template<typename T>
void TestComputeContactSurfaceSoftRigid() {
  auto id_A = GeometryId::get_new_id();
  auto id_B = GeometryId::get_new_id();
  EXPECT_LT(id_A, id_B);
  unique_ptr<VolumeMesh<T>> mesh_S = OctahedronVolume<T>();
  unique_ptr<VolumeMeshFieldLinear<T, T>> field_S =
      OctahedronPressureField<T>(mesh_S.get());
  const Bvh<VolumeMesh<T>> bvh_mesh_S(*mesh_S);
  unique_ptr<SurfaceMesh<T>> surface_R = PyramidSurface<T>();
  const Bvh<SurfaceMesh<T>> bvh_surface_R(*surface_R);
  // Move the rigid pyramid up, so only its square base intersects the top
  // part of the soft octahedron.
  const auto X_SR = RigidTransform<T>(Vector3<T>(0, 0, 0.5));

  // The relationship between the frames for the soft body and the
  // world frame is irrelevant for this test.
  const auto X_WS = RigidTransform<T>::Identity();
  const auto X_WR = X_WS * X_SR;

  // Regardless of how we assign id_A and id_B to mesh_S and surface_R, the
  // contact surfaces will always have id_M = id_A and id_N = id_B (because
  // of the ordering).

  // In this case, we assign id_A to soft and we already know that id_A < id_B.
  // Confirm order
  auto contact_SR = ComputeContactSurfaceFromSoftVolumeRigidSurface(
      id_A, *field_S, bvh_mesh_S, X_WS, id_B, *surface_R, bvh_surface_R, X_WR);
  EXPECT_EQ(contact_SR->id_M(), id_A);
  EXPECT_EQ(contact_SR->id_N(), id_B);
  EXPECT_TRUE(contact_SR->HasGradE_M());
  EXPECT_FALSE(contact_SR->HasGradE_N());

  // Now reverse the ids. It should *still* be the case that the reported id_A
  // is less than id_B, but we should further satisfy various invariants
  // (listed below).
  auto contact_RS = ComputeContactSurfaceFromSoftVolumeRigidSurface(
      id_B, *field_S, bvh_mesh_S, X_WS, id_A, *surface_R, bvh_surface_R, X_WR);
  EXPECT_EQ(contact_RS->id_M(), id_A);
  EXPECT_EQ(contact_RS->id_N(), id_B);
  EXPECT_FALSE(contact_RS->HasGradE_M());
  EXPECT_TRUE(contact_RS->HasGradE_N());

  // Mesh invariants:
  //   Meshes are the same "size" (topologically).
  EXPECT_EQ(contact_SR->mesh_W().num_faces(), contact_RS->mesh_W().num_faces());
  EXPECT_EQ(contact_SR->mesh_W().num_vertices(),
            contact_RS->mesh_W().num_vertices());

  //   Test one and assume all share the same property.
  const SurfaceVertexIndex v_index(0);
  EXPECT_TRUE(CompareMatrices(contact_SR->mesh_W().vertex(v_index).r_MV(),
                              contact_RS->mesh_W().vertex(v_index).r_MV()));

  // TODO(SeanCurtis-TRI): Test that the face winding has been reversed, once
  //  that is officially documented as a property of the ContactSurface.

  // The "pressure" field is frame invariant and should be equal.
  const typename SurfaceMesh<T>::Barycentric centroid(1. / 3., 1. / 3.,
                                                      1. / 3.);
  const SurfaceFaceIndex f_index(0);
  EXPECT_EQ(contact_SR->EvaluateE_MN(f_index, centroid),
            contact_RS->EvaluateE_MN(f_index, centroid));

  // The gradients for the pressure field of the soft mesh are expresssed in the
  // world frame. To determine the world transformation has taken place, we'll
  // find which tetrahedron produced the first triangle in the contact surface.
  // We'll confirm that its gradient has been transformed to the world frame.
  const SurfaceFaceIndex f0(0);
  const VolumeElementIndex t =
      GetTetForTriangle<T>(contact_SR->mesh_W(), f0, *mesh_S, X_WS.inverse());
  EXPECT_TRUE(CompareMatrices(contact_SR->EvaluateGradE_M_W(f0),
                              X_WS.rotation() * field_S->EvaluateGradient(t),
                              std::numeric_limits<double>::epsilon()));
}

GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidDouble) {
  TestComputeContactSurfaceSoftRigid<double>();
}

// Check that we can compile with AutoDiffXd.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidAutoDiffXd) {
  TestComputeContactSurfaceSoftRigid<AutoDiffXd>();
}

// Finds the vertex of the mesh M (SurfaceMesh or VolumeMesh) coincident with
// point Q. It reports the index of the vertex in the mesh. This naively
// performs an exhaustive search and is not suitable for production use. It
// uses a tolerance 1e-14 in checking whether Q and a mesh vertex are
// coincident.
//
// @param[in] p_MQ
//     The position of query point Q measured and expressed in M's frame.
// @param[in] mesh_M
//     The mesh with vertex positions expressed in M's frame.
// @param[out] vertex
//     Index of the mesh's vertex coincident with Q.
// @return
//     true if found.
// @tparam  SurfaceMesh or VolumeMesh
template <class Mesh>
bool FindVertex(Vector3d p_MQ, const Mesh& mesh_M,
                typename Mesh::VertexIndex* vertex) {
  for (typename Mesh::VertexIndex v(0); v < mesh_M.num_vertices(); ++v) {
    if ((p_MQ - mesh_M.vertex(v).r_MV()).norm() < 1e-14) {
      *vertex = v;
      return true;
    }
  }
  return false;
}

// Find the tetrahedral element of the volume mesh M that contains
// a query point Q. It reports the index of the tetrahedral element E and the
// barycentric coordinate b_EQ of Q in the tetrahedron E. It naively performs
// an exhaustive search and is not suitable for production use. It handles
// numerical roundings in limited ways.
//
// @param[in] p_MQ
//     The position of query point Q measured and expressed in M's frame.
// @param[in] volume_M
//     The volume mesh with vertex positions expressed in M's frame.
// @param[out] element_E
//     The index of the tetrahedral element E that contains Q.
// @param[out] b_EQ
//     Barycentric coordinates of the query point Q in the tetrahedral
//     element E.
// @return
//     true if found.
// @note It may incorrectly classify Q slightly outside a tetrahedron as
//       being inside due to numerical roundings.
bool FindElement(Vector3d p_MQ, const VolumeMesh<double>& volume_M,
                 VolumeElementIndex* element_E,
                 VolumeMesh<double>::Barycentric* b_EQ) {
  for (VolumeElementIndex e(0); e < volume_M.num_elements(); ++e) {
    VolumeMesh<double>::Barycentric b = volume_M.CalcBarycentric(p_MQ, e);
    if ((b.array() >= -1e-14).all()) {
      *element_E = e;
      *b_EQ = b;
      return true;
    }
  }
  return false;
}

// Tests ComputeContactSurfaceFromSoftVolumeRigidSurface as we move the rigid
// geometry around. Currently uses double as the scalar type.
// TODO(DamrongGuoy): More comprehensive tests. We should have a better way
//  to check the SurfaceMesh in the output ContactSurface. We should apply
//  general rotations in the pose of N w.r.t. M, instead of 90 degrees turn.
//  We should check the scalar field and the vector field in a more
//  comprehensive way.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidMoving) {
  // Soft octahedron volume S with pressure field.
  auto s_id = GeometryId::get_new_id();
  unique_ptr<VolumeMesh<double>> volume_S = OctahedronVolume<double>();
  const Bvh<VolumeMesh<double>> bvh_volume_S(*volume_S);
  unique_ptr<VolumeMeshFieldLinear<double, double>> pressure_S =
      OctahedronPressureField<double>(volume_S.get());
  // Rigid pyramid surface R.
  auto r_id = GeometryId::get_new_id();
  unique_ptr<SurfaceMesh<double>> surface_R = PyramidSurface<double>();
  const Bvh<SurfaceMesh<double>> bvh_surface_R(*surface_R);

  // We use 1e-14 instead of std::numeric_limits<double>::epsilon() to
  // compensate for the rounding due to general rigid transform.
  const double kEps = 1e-14;

  // Pose of the soft octahedron S in World frame.
  const auto X_WS =
      RigidTransformd(RollPitchYawd(M_PI / 6., 2. * M_PI / 3., M_PI / 4.),
                      Vector3d{1., -0.5, 3.});

  // Tests translation. Set the pose of the rigid pyramid R in S's frame as
  // the one-unit downward translation, so that the apex of the rigid pyramid R
  // is at the center of the soft octahedron S.
  {
    const auto X_SR = RigidTransformd(-Vector3d::UnitZ());
    const auto X_WR = X_WS * X_SR;
    // Contact surface C is expressed in World frame.
    const auto contact = ComputeContactSurfaceFromSoftVolumeRigidSurface(
        s_id, *pressure_S, bvh_volume_S, X_WS, r_id, *surface_R, bvh_surface_R,
        X_WR);
    // TODO(DamrongGuoy): More comprehensive checks on the mesh of the contact
    //  surface. Here we only check the number of triangles.
    EXPECT_EQ(12, contact->mesh_W().num_faces());

    // Point Q is coincident with the center vertex of the soft mesh volume_S.
    // The point C on the contact surface is coincident with Q. Check that the
    // contact surface reports the same pressure at C as the volume does at Q.
    {
      const Vector3d p_SQ = Vector3d::Zero();
      const Vector3d p_WQ = X_WS * p_SQ;
      // Index of contact surface C's vertex coincident with Q.
      SurfaceVertexIndex index_C;
      ASSERT_TRUE(FindVertex(p_WQ, contact->mesh_W(), &index_C));
      const double pressure_at_C = contact->EvaluateE_MN(index_C);
      // Index of Q in the volume mesh.
      VolumeVertexIndex index_Q;
      ASSERT_TRUE(FindVertex(p_SQ, pressure_S->mesh(), &index_Q));
      const double pressure_at_Q = pressure_S->EvaluateAtVertex(index_Q);

      EXPECT_NEAR(pressure_at_C, pressure_at_Q, kEps * pressure_at_Q);
    }
  }

  // Tests rotation. First we rotate the rigid pyramid R 90 degrees around
  // X-axis of the soft octahedron S, so R will fit in the left half, instead
  // of the top half, of S. R's vertices will look like this in S's frame:
  //
  //                +Z   -X
  //                 |   /
  //              v2 ●  ● v3
  //                 | /
  //      v5      v0 |/
  //  -Y---●---------●-----------+Y
  //                /|
  //               / |
  //           v1 ●  ● v4
  //             /   |
  //           +X    |
  //                -Z
  //
  // To avoid the "double counting" problem, we then translate R half a unit
  // length in -Y direction of S's frame.
  //
  // Verify that the contact surface C passes through point Q at (0, -1/2, 0)
  // in S's frame. Notice that Q is coincident with a vertex of C, and Q is
  // on the middle of an edge of a tetrahedron of S.
  //
  {
    const auto X_SR =
        RigidTransformd(RollPitchYawd(M_PI / 2., 0., 0.), Vector3d{0, -0.5, 0});
    const auto X_WR = X_WS * X_SR;
    auto contact = ComputeContactSurfaceFromSoftVolumeRigidSurface(
        s_id, *pressure_S, bvh_volume_S, X_WS, r_id, *surface_R, bvh_surface_R,
        X_WR);
    // TODO(DamrongGuoy): More comprehensive checks on the mesh of the contact
    //  surface.  Here we only check the number of triangles.
    EXPECT_EQ(12, contact->mesh_W().num_faces());

    const Vector3d p_SQ{0, -0.5, 0};
    const Vector3d p_WQ = X_WS * p_SQ;
    // Index of C's vertex coincident with Q.
    SurfaceVertexIndex c_vertex;
    ASSERT_TRUE(FindVertex(p_WQ, contact->mesh_W(), &c_vertex));
    const double c_pressure = contact->EvaluateE_MN(c_vertex);

    // Find the tetrahedral element of S containing Q.
    VolumeElementIndex tetrahedron;
    VolumeMesh<double>::Barycentric b_Q;
    ASSERT_TRUE(FindElement(p_SQ, pressure_S->mesh(), &tetrahedron, &b_Q));
    const double s_pressure = pressure_S->Evaluate(tetrahedron, b_Q);

    EXPECT_NEAR(c_pressure, s_pressure, kEps * s_pressure);
  }
}

// The ultimate proper spelling of mesh intersection allows for mixed scalar
// types (double-valued meshes with autodiff-valued poses). The calling code
// needs to assume this is possible, so we've added a specific overload with
// this spelling. It supports compilation but throws a runtime exception.
// This confirms the exception.
GTEST_TEST(MeshIntersectionTest, DoubleAutoDiffMixed) {
  unique_ptr<VolumeMesh<double>> volume_S = OctahedronVolume<double>();
  const Bvh<VolumeMesh<double>> bvh_volume_S(*volume_S);
  unique_ptr<VolumeMeshFieldLinear<double, double>> field_S =
      OctahedronPressureField<double>(volume_S.get());
  unique_ptr<SurfaceMesh<double>> surface_R = PyramidSurface<double>();
  const Bvh<SurfaceMesh<double>> bvh_surface_R(*surface_R);

  DRAKE_EXPECT_THROWS_MESSAGE(
      ComputeContactSurfaceFromSoftVolumeRigidSurface(
          GeometryId::get_new_id(), *field_S, bvh_volume_S,
          RigidTransform<AutoDiffXd>(), GeometryId::get_new_id(), *surface_R,
          bvh_surface_R, RigidTransform<AutoDiffXd>()),
      std::logic_error,
      "AutoDiff-valued ContactSurface calculation between meshes is not "
      "currently supported");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
