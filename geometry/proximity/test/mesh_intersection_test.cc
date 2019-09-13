#include "drake/geometry/proximity/mesh_intersection.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace mesh_intersection {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;

// TODO(SeanCurtis-TRI): Unit test HalfSpace's signed_distance() and
//  point_is_outside() methods.

// This simply tests arbitrary normals to make sure they satisfy the
// normalization test. They hypothesis is that *any* Vector3 normalized must
// have a magnitude that is less than 1 epsilon away from 1 (except for the
// zero vector, obviously).
GTEST_TEST(HalfSpace, Construction) {
  const double kEps = std::numeric_limits<double>::epsilon();
  std::vector<Vector3d> dirs{
      {1., kEps, kEps},
      {1., std::sqrt(kEps), std::sqrt(kEps)},
      {1., 2 * std::sqrt(kEps), 2 * std::sqrt(kEps)},
      {1.123412345, 10.1231231235, -200.23298298374}
  };
  for (const Vector3d& dir : dirs) {
    // This would abort for normal vectors that aren't unit length (see the
    // death test below).
    HalfSpace<double>(dir.normalized(), 0.25);
  }
}

// Confirms that a plane normal that is *insufficiently* unit length aborts.
GTEST_TEST(HalfSpaceTest, Unnormalized) {
  const double kDelta = 4 * std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_THROW(HalfSpace<double>(Vector3d{1, kDelta, kDelta}, 0.25),
               std::exception);
}

// TODO(SeanCurtis-TRI): Robustly confirm that epsilon of 1e-14 is correct for
//  determining that the intersection is valid. One would suppose that it will
//  depend on the magnitude of the values in play.

// TODO(DamrongGuoy): More comprehensive tests.
GTEST_TEST(MeshIntersectionTest, CalcIntersection) {
  const double kEps = std::numeric_limits<double>::epsilon();
  // TODO(SeanCurtis-TRI): This test has too many zeros in it (the normal is
  //  [1, 0, 0] -- that is not a robust test. Pick a more arbitrary normal.
  // Halfspace {(x,y,z) : x <= 2.0}
  const Vector3d unit_normal_H = Vector3d::UnitX();
  const double plane_offset = 2.0;
  const HalfSpace<double> half_space_H(unit_normal_H, plane_offset);

  // The line AB intersects the plane of the half space.
  {
    const Vector3d p_HA = Vector3d::Zero();
    const Vector3d p_HB(4, 6, 10);
    const Vector3d intersection =
        mesh_intersection::CalcIntersection(p_HA, p_HB, half_space_H);
    const Vector3d expect_intersection(2, 3, 5);
    EXPECT_LE((expect_intersection - intersection).norm(), kEps);
  }

  // The line AB is almost parallel to the plane of the half space.
  {
    const Vector3d p_HA(plane_offset + 2.0 * kEps, 0., 0.);
    const Vector3d p_HB(plane_offset - 2.0 * kEps, 1., 1.);
    const Vector3d intersection =
        mesh_intersection::CalcIntersection(p_HA, p_HB, half_space_H);
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
  // Halfspace {(x,y,z) : x <= 2.0}
  const Vector3d unit_normal = Vector3d::UnitX();
  const double offset = 2.0;
  const HalfSpace<double> half_space(unit_normal, offset);

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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::ClipPolygonByHalfSpace(input_polygon, half_space);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::ClipPolygonByHalfSpace(input_polygon, half_space);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::ClipPolygonByHalfSpace(input_polygon, half_space);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::ClipPolygonByHalfSpace(input_polygon, half_space);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::ClipPolygonByHalfSpace(input_polygon, half_space);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::ClipPolygonByHalfSpace(input_polygon, half_space);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::RemoveDuplicateVertices(input_polygon);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::RemoveDuplicateVertices(input_polygon);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::RemoveDuplicateVertices(input_polygon);
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
    const std::vector<Vector3d> output_polygon =
        mesh_intersection::RemoveDuplicateVertices(input_polygon);
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
std::unique_ptr<SurfaceMesh<T>> TrivialSurfaceMesh() {
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
template<typename T>
std::unique_ptr<VolumeMesh<T>> TrivialVolumeMesh() {
  const int element_data[2][4] = {
      {0, 1, 2, 3},
      {0, 2, 1, 4}};
  std::vector<VolumeElement> elements;
  for (const auto& element : element_data) {
    elements.emplace_back(element);
  }
  const Vector3<T> vertex_data[5] = {
      Vector3<T>::Zero(),
      Vector3<T>::UnitX(),
      Vector3<T>::UnitY(),
      Vector3<T>::UnitZ(),
      -Vector3<T>::UnitZ()
  };
  std::vector<VolumeVertex<T>> vertices;
  for (auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return std::make_unique<VolumeMesh<T>>(std::move(elements),
                                         std::move(vertices));
}

template<typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TrivialVolumeMeshField(
    const VolumeMesh<T>* volume_mesh) {
  // TODO(SeanCurtis-TRI): All the zeros and ones prevent meaningful recognition
  // of valid interpolation. I.e., interpolating values at v0, v1, v2
  // incorrectly will still produce zero. Provide more complex values.

  // Pressure field value pᵢ at vertex vᵢ.
  const T p0{0.};
  const T p1{0.};
  const T p2{0.};
  const T p3{1.};
  const T p4{1.};
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
    const auto polygon = mesh_intersection::ClipTriangleByTetrahedron(
        element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3d> expect_empty_polygon;
    EXPECT_TRUE(CompareConvexPolygon(expect_empty_polygon, polygon));
  }

  // The triangle is outside the tetrahedron `element0` with one edge on a
  // face of the tetrahedron. Expect the output polygon to be empty.
  {
    const auto X_MN = RigidTransformd(RollPitchYawd(0, 0, M_PI_2),
                                             Vector3d::Zero());
    const auto polygon = mesh_intersection::ClipTriangleByTetrahedron(
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
    const auto polygon0_M = mesh_intersection::ClipTriangleByTetrahedron(
        element0, *volume_M, face, *surface_N, X_MN);
    const auto polygon1_M = mesh_intersection::ClipTriangleByTetrahedron(
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
    const auto polygon0_M = mesh_intersection::ClipTriangleByTetrahedron(
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
    const auto polygon1_M = mesh_intersection::ClipTriangleByTetrahedron(
        element1, *volume_M, face, *surface_N, X_MN);
    EXPECT_TRUE(CompareConvexPolygon(empty_polygon, polygon1_M));
  }

  // The triangle intersects the tetrahedron `element0` such that the result is
  // a quad.
  {
    const auto X_MN = RigidTransformd(RollPitchYawd(0, 0, M_PI),
                                             Vector3d(0.5, 0.5, 0));
    const auto polygon0_M = mesh_intersection::ClipTriangleByTetrahedron(
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
  std::unique_ptr<VolumeMesh<double>> volume_M;
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
  std::unique_ptr<SurfaceMesh<double>> surface_N;
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
  const auto polygon_M = mesh_intersection::ClipTriangleByTetrahedron(
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

// TODO(DamrongGuoy): Add unit tests for AddPolygonToMeshData().

// TODO(DamrongGuoy): Add unit tests for ComputeNormalField().

// TODO(DamrongGuoy): Test SampleVolumeFieldOnSurface with more general
//  X_MN.  Right now X_MN is a simple translation without rotation.

GTEST_TEST(MeshIntersectionTest, SampleVolumeFieldOnSurface) {
  auto volume_M = TrivialVolumeMesh<double>();
  auto volume_field_M = TrivialVolumeMeshField<double>(volume_M.get());
  auto rigid_N = TrivialSurfaceMesh<double>();
  const auto X_MN = RigidTransformd(Vector3d(0, 0, 0.5));

  std::unique_ptr<SurfaceMesh<double>> surface;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_field;
  std::unique_ptr<SurfaceMeshFieldLinear<Vector3d, double>> grad_h_field;
  mesh_intersection::SampleVolumeFieldOnSurface(
      *volume_field_M, *rigid_N, X_MN,
      &surface, &e_field, &grad_h_field);

  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_EQ(1, surface->num_faces());
  // TODO(DamrongGuoy): More comprehensive checks.
  const double area = surface->area(SurfaceFaceIndex(0));
  const double expect_area = (1. / 2.) * 0.5 * 0.5;
  EXPECT_NEAR(expect_area, area, kEps);
  const SurfaceFaceIndex face0(0);
  const SurfaceMesh<double>::Barycentric centroid(1. / 3., 1. / 3., 1. / 3.);
  const double e = e_field->Evaluate(face0, centroid);
  const double expect_e = 0.5;
  EXPECT_NEAR(expect_e, e, kEps);
  const auto grad_h = grad_h_field->Evaluate(face0, centroid);
  const auto expect_grad_h = Vector3d::UnitZ();
  EXPECT_NEAR((grad_h - expect_grad_h).norm(), 0., kEps);
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
std::unique_ptr<VolumeMesh<T>> OctahedronVolume() {
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
std::unique_ptr<VolumeMeshFieldLinear<T, T>> OctahedronPressureField(
    VolumeMesh<T>* volume_mesh) {
  // The field is 0 on the boundary and linearly increasing to 1 at the
  // center of the octahedron.
  std::vector<T> values{1, 0, 0, 0, 0, 0, 0};
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
std::unique_ptr<SurfaceMesh<T>> PyramidSurface() {
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
  auto mesh_S = OctahedronVolume<T>();
  auto field_S = OctahedronPressureField<T>(mesh_S.get());
  auto surface_R = PyramidSurface<T>();
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
  auto contact_SR =
      mesh_intersection::ComputeContactSurfaceFromSoftVolumeRigidSurface(
          id_A, *field_S, X_WS, id_B, *surface_R, X_WR);
  EXPECT_EQ(contact_SR->id_M(), id_A);
  EXPECT_EQ(contact_SR->id_N(), id_B);

  // Now reverse the ids. It should *still* be the case that the reported id_A
  // is less than id_B, but we should further satisfy various invariants
  // (listed below).
  auto contact_RS =
      mesh_intersection::ComputeContactSurfaceFromSoftVolumeRigidSurface(
          id_B, *field_S, X_WS, id_A, *surface_R, X_WR);
  EXPECT_EQ(contact_RS->id_M(), id_A);
  EXPECT_EQ(contact_RS->id_N(), id_B);

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

  // The gradient fields are related by only a reflection around the origin.
  EXPECT_TRUE(CompareMatrices(
      contact_SR->EvaluateGrad_h_MN_W(f_index, centroid),
      -contact_RS->EvaluateGrad_h_MN_W(f_index, centroid)));
}

GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidDouble) {
  TestComputeContactSurfaceSoftRigid<double>();
}

// Check that we can compile with AutoDiffXd.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidAutoDiffXd) {
  TestComputeContactSurfaceSoftRigid<AutoDiffXd>();
}

// Utility to find the vertex of the surface mesh M coincident with point Q. It
// reports the index of an incident face and the barycentric coordinates of that
// vertex. This naively performs an exhaustive search and is not suitable for
// production use and requires the point to be bit-identical to `p_MQ`.
// @param[in] p_MQ
//     The position of query point Q measured and expressed in M's frame.
// @param[in] surface_M
//     The surface mesh with vertex positions expressed in M's frame.
// @param[out] face
//     The index of a face incident to the coincindent vertex.
// @param[out] vertex
//     Barycentric coordinates of the vertex in the face. It would be either
//     (1,0,0) or (0,1,0) or (0,0,1) depending on which vertex in the face
//     matches `p_MQ`.
// @return
//     true if found.
bool FindFaceVertex(Vector3d p_MQ, const SurfaceMesh<double>& surface_M,
                    SurfaceFaceIndex* face,
                    SurfaceMesh<double>::Barycentric* vertex) {
  for (SurfaceFaceIndex f(0); f < surface_M.num_faces(); ++f) {
    for (int i = 0; i < 3; ++i) {
      const SurfaceVertexIndex v = surface_M.element(f).vertex(i);
      if (p_MQ == surface_M.vertex(v).r_MV()) {
        *face = f;
        *vertex = SurfaceMesh<double>::Barycentric::Zero();
        (*vertex)(i) = 1.;
        return true;
      }
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
  auto id_S = GeometryId::get_new_id();
  auto id_R = GeometryId::get_new_id();
  auto soft_mesh = OctahedronVolume<double>();
  // TODO(edrumwri) Fix the disparity here: OctahedronPressureField claims to
  // be a pressure field but it is treated like a strain field.
  auto soft_epsilon = OctahedronPressureField<double>(soft_mesh.get());
  auto rigid_mesh = PyramidSurface<double>();

  const double kEps = std::numeric_limits<double>::epsilon();

  // The relationship between the frames for the soft body and the
  // world frame is irrelevant for this test.
  const auto X_WS = RigidTransformd::Identity();

  // Tests translation. Move the rigid pyramid down, so its apex is at the
  // center of the soft octahedron.  Check the field values at that point.
  // We expect that the contact surface must include the zero vertex.
  {
    const auto X_SR = RigidTransformd(-Vector3d::UnitZ());
    const auto X_WR = X_WS * X_SR;
    auto contact_SR_W =
        mesh_intersection::ComputeContactSurfaceFromSoftVolumeRigidSurface(
            id_S, *soft_epsilon, X_WS, id_R, *rigid_mesh, X_WR);
    // TODO(DamrongGuoy): More comprehensive checks on the mesh of the contact
    //  surface. Here we only check the number of triangles.
    EXPECT_EQ(4, contact_SR_W->mesh_W().num_faces());

    const Vector3d p_MQ = Vector3d::Zero();
    SurfaceFaceIndex face_Q;
    SurfaceMesh<double>::Barycentric b_Q;
    bool found = FindFaceVertex(p_MQ, contact_SR_W->mesh_W(), &face_Q, &b_Q);
    ASSERT_TRUE(found);
    const auto epsilon_SR = contact_SR_W->EvaluateE_MN(face_Q, b_Q);
    EXPECT_NEAR(1.0, epsilon_SR, kEps);
    const auto grad_h_W = contact_SR_W->EvaluateGrad_h_MN_W(face_Q, b_Q);
    const Vector3d expect_grad_h_W = Vector3d::UnitZ();
    EXPECT_TRUE(CompareMatrices(expect_grad_h_W, grad_h_W, kEps));
  }
  // Tests rotation. First we rotate the rigid pyramid 90 degrees around
  // X-axis, so it will fit the left half of the soft octahedron, instead of
  // the top half of the octahedron.  The pyramid vertices will look like this:
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
  //
  // To  avoid "double counting" problem, we then translate the pyramid a bit in
  // the -Y direction. The center of the contact surface will be at (0, -1/2, 0)
  // in the soft octahedron's frame.
  {
    const auto X_SR =
        RigidTransformd(RollPitchYawd(M_PI / 2., 0., 0.), Vector3d{0, -0.5, 0});
    const auto X_WR = X_WS * X_SR;
    auto contact_SR_W =
        mesh_intersection::ComputeContactSurfaceFromSoftVolumeRigidSurface(
            id_S, *soft_epsilon, X_WS, id_R, *rigid_mesh, X_WR);
    // TODO(DamrongGuoy): More comprehensive checks on the mesh of the contact
    //  surface.  Here we only check the number of triangles.
    EXPECT_EQ(4, contact_SR_W->mesh_W().num_faces());

    const Vector3d p_MQ{0, -0.5,
                        0};  // The center vertex of the pyramid "bottom".
    SurfaceFaceIndex face_Q;
    SurfaceMesh<double>::Barycentric b_Q;
    bool found = FindFaceVertex(p_MQ, contact_SR_W->mesh_W(), &face_Q, &b_Q);
    ASSERT_TRUE(found);
    const auto e_SR = contact_SR_W->EvaluateE_MN(face_Q, b_Q);
    EXPECT_NEAR(0.5, e_SR, kEps);
    const auto grad_h_W = contact_SR_W->EvaluateGrad_h_MN_W(face_Q, b_Q);
    const Vector3d expect_grad_h_W = X_WS * Vector3d::UnitY();
    EXPECT_NEAR((expect_grad_h_W - grad_h_W).norm(), 0., kEps);
  }
}

}  // namespace
}  // namespace mesh_intersection
}  // namespace geometry
}  // namespace drake
