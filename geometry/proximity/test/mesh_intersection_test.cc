#include "drake/geometry/proximity/mesh_intersection.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

/* @file Tests the the main function
 ComputeContactSurfaceFromSoftVolumeRigidSurface and it's supporting code. The
 main function can produce ContactSurfaces with either triangle or polygon
 representations. However, not all tests need to be exercised against both
 possible representations. Each test is documented as to whether it needs to
 consider the two possible representations or if the code under test is
 orthogonal to that choice. */

namespace drake {
namespace geometry {
namespace internal {

template<typename MeshType>
class SurfaceVolumeIntersectorTester {
 public:
  using T = typename MeshType::ScalarType;

  Vector3<T> CalcIntersection(const Vector3<T>& p_FA, const Vector3<T>& p_FB,
                              const PosedHalfSpace<double>& H_F) {
    return intersect_.CalcIntersection(p_FA, p_FB, H_F);
  }
  void ClipPolygonByHalfSpace(const std::vector<Vector3<T>>& polygon_vertices_F,
                              const PosedHalfSpace<double>& H_F,
                              std::vector<Vector3<T>>* output_vertices_F) {
    intersect_.ClipPolygonByHalfSpace(polygon_vertices_F, H_F,
                                      output_vertices_F);
  }
  void RemoveDuplicateVertices(std::vector<Vector3<T>>* polygon) {
    intersect_.RemoveDuplicateVertices(polygon);
  }
  const std::vector<Vector3<T>>& ClipTriangleByTetrahedron(
      int element, const VolumeMesh<double>& volume_M, int face,
      const TriangleSurfaceMesh<double>& surface_N,
      const math::RigidTransform<T>& X_MN) {
    return intersect_.ClipTriangleByTetrahedron(element, volume_M, face,
                                                surface_N, X_MN);
  }
  bool IsFaceNormalAlongPressureGradient(
      const VolumeMeshFieldLinear<double, double>& volume_field_M,
      const TriangleSurfaceMesh<double>& surface_N,
      const math::RigidTransform<T>& X_MN, int tet_index, int tri_index) {
    return intersect_.IsFaceNormalAlongPressureGradient(
        volume_field_M, surface_N, X_MN, tet_index, tri_index);
  }

 private:
  SurfaceVolumeIntersector<MeshType> intersect_;
};

namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_unique;
using std::pair;
using std::unique_ptr;
using std::vector;

// TODO(SeanCurtis-TRI): Robustly confirm that epsilon of 1e-14 is correct for
//  determining that the intersection is valid. One would suppose that it will
//  depend on the magnitude of the values in play.

// TODO(DamrongGuoy): More comprehensive tests.
/* This test is independent of ContactSurface mesh representation; so we'll
 simply use TriangleSurfaceMesh. */
GTEST_TEST(MeshIntersectionTest, CalcIntersection) {
  using MeshType = TriangleSurfaceMesh<double>;

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
        SurfaceVolumeIntersectorTester<MeshType>().CalcIntersection(
            p_HA, p_HB, half_space_H);
    const Vector3d expect_intersection(2, 3, 5);
    EXPECT_LE((expect_intersection - intersection).norm(), kEps);
  }

  // The line AB is almost parallel to the plane of the half space.
  {
    const Vector3d p_HA(plane_offset + 2.0 * kEps, 0., 0.);
    const Vector3d p_HB(plane_offset - 2.0 * kEps, 1., 1.);
    const Vector3d intersection =
        SurfaceVolumeIntersectorTester<MeshType>().CalcIntersection(
            p_HA, p_HB, half_space_H);
    const Vector3d expect_intersection(2., 0.5, 0.5);
    EXPECT_LE((expect_intersection - intersection).norm(), kEps);
  }

  // TODO(SeanCurtis-TRI): Confirm death test in debug mode for if the points
  //  don't properly "straddle" the boundary plane.
  //  - Both on negative, both on positive, both *on* the plane.
  //  - parallel negative, parallel positive.
}

// TODO(DamrongGuoy): Move the definition of this function here after 11612
//  landed. The definition is currently down below here.
template<typename T>
bool CompareConvexPolygon(const std::vector<Vector3<T>>& polygon0,
                          const std::vector<Vector3<T>>& polygon1);

// Although polygons with zero, one, or two vertices are valid input and are
// treated correctly by this function, they are not tested as being a
// meaningless operation. This test is independent of ContactSurface mesh
// representation; so we'll simply use TriangleSurfaceMesh.
GTEST_TEST(MeshIntersectionTest, ClipPolygonByHalfSpace) {
  using MeshType = TriangleSurfaceMesh<double>;

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
    SurfaceVolumeIntersectorTester<MeshType>().ClipPolygonByHalfSpace(
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
    SurfaceVolumeIntersectorTester<MeshType>().ClipPolygonByHalfSpace(
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
    SurfaceVolumeIntersectorTester<MeshType>().ClipPolygonByHalfSpace(
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
    SurfaceVolumeIntersectorTester<MeshType>().ClipPolygonByHalfSpace(
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
    SurfaceVolumeIntersectorTester<MeshType>().ClipPolygonByHalfSpace(
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
    SurfaceVolumeIntersectorTester<MeshType>().ClipPolygonByHalfSpace(
        input_polygon, half_space_H, &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_output_polygon, output_polygon));
  }
  // TODO(SeanCurtis-TRI): Clip a triangle into a quad. Clip a triangle into a
  // triangle.
}

/* This test is independent of ContactSurface mesh representation; so we'll
 simply use TriangleSurfaceMesh. */
GTEST_TEST(MeshIntersectionTest, RemoveDuplicateVertices) {
  using MeshType = TriangleSurfaceMesh<double>;

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
    SurfaceVolumeIntersectorTester<MeshType>().RemoveDuplicateVertices(
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
    SurfaceVolumeIntersectorTester<MeshType>().RemoveDuplicateVertices(
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
    SurfaceVolumeIntersectorTester<MeshType>().RemoveDuplicateVertices(
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
    SurfaceVolumeIntersectorTester<MeshType>().RemoveDuplicateVertices(
        &output_polygon);
    EXPECT_TRUE(CompareConvexPolygon(expect_three_vertices, output_polygon));
  }
}

// TODO(SeanCurtis-TRI): Too many zeros; it reduces the power of the test
//  because so much of the math simply disappears. Reframe these tests with
//  non-trivial triangles and tets -- the recommendation is to formulate these
//  in an easy frame, find the answer in the easy frame, and then transform
//  them all into an obnoxious frame and test the code there. This applies to
//  both triangle and tetrahedron.

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
unique_ptr<TriangleSurfaceMesh<T>> TrivialSurfaceMesh() {
  const int face_data[3] = {0, 1, 2};
  std::vector<SurfaceTriangle> faces{SurfaceTriangle(face_data)};
  std::vector<Vector3<T>> vertices = {
      Vector3<T>::Zero(),
      Vector3<T>::UnitX(),
      Vector3<T>::UnitY()
  };
  return std::make_unique<TriangleSurfaceMesh<T>>(std::move(faces),
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
  std::vector<Vector3<T>> vertices = {
      X_MN * Vector3<T>::Zero(),
      X_MN * Vector3<T>::UnitX(),
      X_MN * Vector3<T>::UnitY(),
      X_MN * Vector3<T>::UnitZ(),
      X_MN * (-Vector3<T>::UnitZ())
  };
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
      std::move(p_values), volume_mesh);

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

/* This test is independent of ContactSurface mesh representation; so we'll
 simply use TriangleSurfaceMesh. */
GTEST_TEST(MeshIntersectionTest, ClipTriangleByTetrahedron) {
  using MeshType = TriangleSurfaceMesh<double>;

  auto volume_M = TrivialVolumeMesh<double>();
  auto surface_N = TrivialSurfaceMesh<double>();
  // TODO(SeanCurtis-TRI): Seeing these two types together suggests there's a
  //  naming problem. Volume *element* and surface *face*. Why is it not
  //  surface element? Element doesn't imply 3D. Alternatively, I favor
  //  TetrahedronIndex and TriangleIndex (or TetIndex and TriIndex,
  //  respectively) as being far more literal. It's not like each of those
  //  surface types are ever designed to support any other kind of "element".
  const int element0 = 0;
  const int element1 = 1;
  const int face = 0;
  const std::vector<Vector3d> empty_polygon;

  // The triangle is outside the tetrahedron `element0` with one vertex on a
  // face of the tetrahedron. Expect the output polygon to be empty.
  {
    const auto X_MN = RigidTransformd(Vector3d::UnitX());
    const std::vector<Vector3d> polygon =
        SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
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
        SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
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
        SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
            element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3d> polygon1_M =
        SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
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
        SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
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
        SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
            element1, *volume_M, face, *surface_N, X_MN);
    EXPECT_TRUE(CompareConvexPolygon(empty_polygon, polygon1_M));
  }

  // The triangle intersects the tetrahedron `element0` such that the result is
  // a quad.
  {
    const auto X_MN = RigidTransformd(RollPitchYawd(0, 0, M_PI),
                                             Vector3d(0.5, 0.5, 0));
    const std::vector<Vector3d> polygon0_M =
        SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
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
// This test is independent of ContactSurface mesh representation; so we'll
// simply use TriangleSurfaceMesh.
GTEST_TEST(MeshIntersectionTest, ClipTriangleByTetrahedronIntoHeptagon) {
  using MeshType = TriangleSurfaceMesh<double>;

  unique_ptr<VolumeMesh<double>> volume_M;
  {
    const int element_data[4] = {0, 1, 2, 3};
    std::vector<VolumeElement> elements{VolumeElement(element_data)};
    // clang-format off
    std::vector<Vector3d> vertices = {
        { 2,  0,  2},
        {-2,  0,  2},
        { 0,  2, -2},
        { 0, -2, -2}
    };
    // clang-format on
    volume_M = std::make_unique<VolumeMesh<double>>(std::move(elements),
                                                    std::move(vertices));
  }
  unique_ptr<TriangleSurfaceMesh<double>> surface_N;
  {
    const int face_data[3] = {0, 1, 2};
    std::vector<SurfaceTriangle> faces{SurfaceTriangle(face_data)};
    // clang-format off
    std::vector<Vector3d> vertices = {
        {1.5,   1.5, 0.},
        {-1.5,  0.,  0.},
        {0.,   -1.5, 0.}};
    // clang-format on
    surface_N = std::make_unique<TriangleSurfaceMesh<double>>(std::move(faces),
                                                      std::move(vertices));
  }
  const int tetrahedron = 0;
  const int triangle = 0;
  const auto X_MN = RigidTransformd::Identity();
  const std::vector<Vector3d> polygon_M =
      SurfaceVolumeIntersectorTester<MeshType>().ClipTriangleByTetrahedron(
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

/* This test is independent of ContactSurface mesh representation; so we'll
 simply use TriangleSurfaceMesh. */
GTEST_TEST(MeshIntersectionTest, IsFaceNormalAlongPressureGradient) {
  using MeshType = TriangleSurfaceMesh<double>;

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

  // We will set the pose of TriangleSurfaceMesh N in frame M so that triangle
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
    EXPECT_EQ(t.expect_result, SurfaceVolumeIntersectorTester<MeshType>()
                                   .IsFaceNormalAlongPressureGradient(
                                       *volume_field_M, *rigid_N, X_MN,
                                       0 /*tet_index*/, 0 /*tri_index*/));
  }
}

// Given a face in a surface mesh, reports the tet in the volume mesh that
// completely contains the face. Throws if a test cannot be identified.
template <typename MeshType>
int GetTetForFace(
    const MeshType& surface_S, int f, const VolumeMesh<double>& volume_V,
    const RigidTransform<typename MeshType::ScalarType>& X_VS) {
  using T = typename MeshType::ScalarType;

  // Each face lies completely within one tet in the volume mesh. The
  // gradient value reported should be that of that tet. So, we'll grab
  // a point inside each face (mean point of the first three vertices),
  // and compute the barycentric coordinate of that point in the tet. The tet
  // that computes a "valid" barycentric coordinate, is the tet that contains
  // the polygon.
  const auto& face = surface_S.element(f);
  const Vector3<T> p_SC =
      (surface_S.vertex(face.vertex(0)) + surface_S.vertex(face.vertex(1)) +
       surface_S.vertex(face.vertex(2))) /
      3;
  const Vector3<T> p_VC = X_VS * p_SC;
  for (int e = 0; e < volume_V.num_elements(); ++e) {
    auto bary_e = volume_V.CalcBarycentric(p_VC, e);
    if ((bary_e.array() >= 0).all() && (bary_e.array() <= 1).all()) {
      return e;
    }
  }
  throw std::logic_error(fmt::format(
      "Surface triangle was unable to place triangle {} in any tetrahedron",
      f));
}

template <typename MeshType>
using MeshBuilderForMesh = std::conditional_t<
    std::is_same_v<MeshType,
                   TriangleSurfaceMesh<typename MeshType::ScalarType>>,
    TriMeshBuilder<typename MeshType::ScalarType>,
    PolyMeshBuilder<typename MeshType::ScalarType>>;

// This fixture will test SampleVolumeFieldOnSurface() and
// ComputeContactSurfaceFromSoftVolumeRigidSurface(). The latter is the main API
// of this module and calls the former. They share many parameters. This
// fixture uses `double` for checking the algorithm. There is another fixture
// that uses `AutoDiffXd` for checking derivatives.
template <typename MeshType>
class MeshIntersectionFixture : public testing::Test {
 protected:
  void SetUp() override {
    // The soft volume mesh is expressed in frame S.
    mesh_S_ = TrivialVolumeMesh<double>();
    bvh_mesh_S_ = make_unique<Bvh<Obb, VolumeMesh<double>>>(*mesh_S_);
    field_S_ = TrivialVolumeMeshField<double>(mesh_S_.get());
    // The rigid surface mesh is expressed in frame R.
    surface_R_ = TrivialSurfaceMesh<double>();
    bvh_surface_R_ =
        make_unique<Bvh<Obb, TriangleSurfaceMesh<double>>>(*surface_R_);
    // Transform the surface (single triangle) so that it intersects with *both*
    // tets in the volume mesh. The surface lies on the y = 0.75 plane.
    // Each tet gets intersected into a isosceles right triangle with a leg
    // length of 0.25m.
    const auto R_SR = RotationMatrixd::MakeXRotation(M_PI_2);
    X_SR_ = RigidTransformd(R_SR, Vector3d(-0.1, 0.75, -0.25));
  }

  // This helper function verifies the output (surface_S, e_field, and
  // grad_eS_S) of SampleVolumeFieldOnSurface().
  void VerifySampleVolumeFieldOnSurface(
      const MeshType& surface_S,
      const typename SurfaceVolumeIntersector<MeshType>::FieldType& e_field,
      const vector<Vector3<double>>& grad_eS_S) {
    // The two geometries intersect such that both tets get sliced into
    // identical right, isosceles triangles (with a leg length of 0.25m). The
    // total area is 2 * 0.25**2 / 2 = 0.25**2.
    const double expect_area = 0.25 * 0.25;
    EXPECT_NEAR(expect_area, surface_S.total_area(), kEps);

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
    bool domain_checked[] = {false, false, false};
    for (int v = 0; v < surface_S.num_vertices(); ++v) {
      const double p_SV_z = surface_S.vertex(v)[2];
      if (std::abs(p_SV_z) < kEps) {
        ASSERT_NEAR(e_field.EvaluateAtVertex(v), 0.0, kEpsPressure);
        domain_checked[0] = true;
      } else if (std::abs(p_SV_z - 0.25) < kEps) {
        ASSERT_NEAR(e_field.EvaluateAtVertex(v), 0.25 * 1e7, kEpsPressure);
        domain_checked[1] = true;
      } else if (std::abs(p_SV_z + 0.25) < kEps) {
        ASSERT_NEAR(e_field.EvaluateAtVertex(v), 0.25 * 1e10, kEpsPressure);
        domain_checked[2] = true;
      }
    }
    // Confirm the e_field tests didn't pass by omission.
    ASSERT_TRUE(domain_checked[0])
                  << "Assumptions have been broken! In testing e_field, no "
                     "vertex was on the z = 0 plane.";
    ASSERT_TRUE(domain_checked[1])
                  << "Assumptions have been broken! In testing e_field, no "
                     "vertex was located at z = 0.25";
    ASSERT_TRUE(domain_checked[2])
                  << "Assumptions have been broken! In testing e_field, no "
                     "vertex was located at z = -0.25";

    // Test the face normals of resulting mesh. Because the 'trivial' surface
    // mesh is a single triangle, all triangles in the resulting mesh should
    // have the same normal.
    ASSERT_TRUE(
        CompareMatrices(surface_R_->face_normal(0), Vector3d::UnitZ()));
    for (int f = 0; f < surface_S.num_elements(); ++f) {
      EXPECT_TRUE(CompareMatrices(surface_S.face_normal(f),
                                  X_SR_.rotation() * Vector3d::UnitZ(),
                                  4 * kEps));
    }

    // Only the soft volume mesh provides gradients.
    ASSERT_EQ(surface_S.num_elements(), static_cast<int>(grad_eS_S.size()));
    for (int f = 0; f < surface_S.num_elements(); ++f) {
      const int t = GetTetForFace(surface_S, f, *mesh_S_, {});
      ASSERT_TRUE(
          CompareMatrices(grad_eS_S[f], field_S_->EvaluateGradient(t)));
    }
    // By design, we wanted to have *different* pressure gradients present
    // in the mesh (hence the reason for intersecting both tetrahedra). Let's
    // confirm that is the case. We assume the first and last triangles in
    // the surface are from *different* tetrahedra. The pressure increases
    // as we move away from the z = 0 plane. So, they'll have different signs
    // and compare as different with a *massive* tolerance (here equal to the
    // maximum pressure value).
    EXPECT_FALSE(CompareMatrices(grad_eS_S.front(), grad_eS_S.back(), 1e10));
  }

  // This helper function verifies consistency between two output from calling
  // ComputeContactSurfaceFromSoftVolumeRigidSurface() twice with different
  // orders of GeometryIds.
  void VerifyComputeContactSurfaceFromSoftRigid(
      const ContactSurface<double>& contact_SR,
      const ContactSurface<double>& contact_RS,
      const RigidTransformd& X_WS) {
    using FieldType = typename SurfaceVolumeIntersector<MeshType>::FieldType;
    const MeshType* mesh_SR_raw{};
    const FieldType* field_SR_raw{};
    const MeshType* mesh_RS_raw{};
    const FieldType* field_RS_raw{};
    if constexpr (std::is_same_v<
                      MeshType,
                      TriangleSurfaceMesh<typename MeshType::ScalarType>>) {
      mesh_SR_raw = &contact_SR.tri_mesh_W();
      field_SR_raw = &contact_SR.tri_e_MN();
      mesh_RS_raw = &contact_RS.tri_mesh_W();
      field_RS_raw = &contact_RS.tri_e_MN();
    } else {
      mesh_SR_raw = &contact_SR.poly_mesh_W();
      field_SR_raw = &contact_SR.poly_e_MN();
      mesh_RS_raw = &contact_RS.poly_mesh_W();
      field_RS_raw = &contact_RS.poly_e_MN();
    }
    const MeshType& mesh_SR_W = *mesh_SR_raw;
    const FieldType& field_SR_W = *field_SR_raw;
    const MeshType& mesh_RS_W = *mesh_RS_raw;
    const FieldType& field_RS_W = *field_RS_raw;

    // Mesh invariants:
    // Meshes are the same "size" (topologically).
    EXPECT_EQ(mesh_SR_W.num_elements(), mesh_RS_W.num_elements());
    EXPECT_EQ(mesh_SR_W.num_vertices(), mesh_RS_W.num_vertices());

    // Test one and assume all share the same property.
    EXPECT_TRUE(CompareMatrices(mesh_SR_W.vertex(0), mesh_RS_W.vertex(0)));

    // TODO(SeanCurtis-TRI): Test that the face winding has been reversed, once
    //  that is officially documented as a property of the ContactSurface.

    // The "pressure" field is frame invariant and should be equal.
    // Pick a point to test the pressure field evaluation.
    const auto& face = mesh_SR_W.element(0);
    const Vector3<double> p_WV =
        (mesh_SR_W.vertex(face.vertex(0)) + mesh_SR_W.vertex(face.vertex(1)) +
         mesh_SR_W.vertex(face.vertex(2))) /
        3;
    const int f_index = 0;
    EXPECT_EQ(field_SR_W.EvaluateCartesian(f_index, p_WV),
              field_RS_W.EvaluateCartesian(f_index, p_WV));

    // The gradients for the pressure field of the soft mesh are expressed in
    // the world frame. To determine the world transformation has taken place,
    // we'll find which tetrahedron produced the first triangle in the contact
    // surface. We'll confirm that its gradient has been transformed to the
    // world frame.
    const int f0 = 0;
    const int t = GetTetForFace(mesh_SR_W, f0, *mesh_S_, X_WS.inverse());
    EXPECT_TRUE(CompareMatrices(contact_SR.EvaluateGradE_M_W(f0),
                                X_WS.rotation() * field_S_->EvaluateGradient(t),
                                4. * kEps));
  }

  // Soft volume mesh with pressure field.
  unique_ptr<VolumeMesh<double>> mesh_S_;
  unique_ptr<Bvh<Obb, VolumeMesh<double>>> bvh_mesh_S_;
  unique_ptr<VolumeMeshFieldLinear<double, double>> field_S_;

  // Rigid surface mesh.
  unique_ptr<TriangleSurfaceMesh<double>> surface_R_;
  unique_ptr<Bvh<Obb, TriangleSurfaceMesh<double>>> bvh_surface_R_;

  RigidTransformd X_SR_;

  static constexpr double kEps = std::numeric_limits<double>::epsilon();
};

using MeshTypes =
    ::testing::Types<TriangleSurfaceMesh<double>, PolygonSurfaceMesh<double>>;
TYPED_TEST_SUITE(MeshIntersectionFixture, MeshTypes);

// A simple smoke test. We perform the collision, confirm the face count of the
// contact surface's mesh, and then confirm that the we've sampled the compliant
// mesh's pressure field.
TYPED_TEST(MeshIntersectionFixture, SampleVolumeFieldOnSurface) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  SurfaceVolumeIntersector<MeshType> intersector;
  intersector.SampleVolumeFieldOnSurface(
      *this->field_S_, *this->bvh_mesh_S_, *this->surface_R_,
      *this->bvh_surface_R_, this->X_SR_, MeshBuilderForMesh<MeshType>());

  const auto& surface_S = intersector.mutable_mesh();
  // The two meshes intersected forming two triangles. Each mesh type responds
  // slightly differently.
  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    // Two triangles tesselate around centroids into six in TriangleSurfaceMesh.
    EXPECT_EQ(6, surface_S.num_elements());
  } else {
    // Two triangles added directly in PolygonSurfaceMesh.
    EXPECT_EQ(2, surface_S.num_elements());
  }
  this->VerifySampleVolumeFieldOnSurface(surface_S, intersector.mutable_field(),
                                         intersector.mutable_grad_eM_M());
}


// Tests the generation of the ContactSurface between a soft volume and rigid
// surface. This highest-level function's primary responsibility is to make
// sure that the resulting ContactSurface satisfies the invariant id_M < id_N.
// To that end, it computes the contact surface twice, with the ids reversed
// and confirms the results reflect that: (i.e., vertex positions are different,
// gradients are mirrored.) The difference test is coarsely sampled and assumes
// that some good results are correlated with all good results based on the
// unit tests for ContactSurface.
TYPED_TEST(MeshIntersectionFixture, TestComputeContactSurfaceSoftRigid) {
  const auto id_A = GeometryId::get_new_id();
  const auto id_B = GeometryId::get_new_id();
  EXPECT_LT(id_A, id_B);
  // The relationship between the frames for the soft body and the
  // world frame is irrelevant for this test.
  const RigidTransformd X_WS = RigidTransformd::Identity();
  const RigidTransformd X_WR = X_WS * this->X_SR_;

  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  HydroelasticContactRepresentation representation =
      HydroelasticContactRepresentation::kTriangle;
  if (!std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    representation = HydroelasticContactRepresentation::kPolygon;
  }

  // Regardless of how we assign id_A and id_B to mesh_S_ and surface_R_, the
  // contact surfaces will always have id_M = id_A and id_N = id_B (because
  // of the ordering).

  // In this case, we assign id_A to soft and we already know that
  // id_A < id_B. Confirm order
  auto contact_SR = ComputeContactSurfaceFromSoftVolumeRigidSurface(
      id_A, *this->field_S_, *this->bvh_mesh_S_, X_WS, id_B, *this->surface_R_,
      *this->bvh_surface_R_, X_WR, representation);
  EXPECT_EQ(contact_SR->id_M(), id_A);
  EXPECT_EQ(contact_SR->id_N(), id_B);
  EXPECT_TRUE(contact_SR->HasGradE_M());
  EXPECT_FALSE(contact_SR->HasGradE_N());

  // Now reverse the ids. It should *still* be the case that the reported id_A
  // is less than id_B, but we should further satisfy various invariants
  // in VerifyComputeContactSurfaceFromSoftRigid().
  auto contact_RS = ComputeContactSurfaceFromSoftVolumeRigidSurface(
      id_B, *this->field_S_, *this->bvh_mesh_S_, X_WS, id_A, *this->surface_R_,
      *this->bvh_surface_R_, X_WR, representation);
  EXPECT_EQ(contact_RS->id_M(), id_A);
  EXPECT_EQ(contact_RS->id_N(), id_B);
  EXPECT_FALSE(contact_RS->HasGradE_M());
  EXPECT_TRUE(contact_RS->HasGradE_N());

  this->VerifyComputeContactSurfaceFromSoftRigid(*contact_SR, *contact_RS,
                                                 X_WS);
}

/* This test fixture enables some limited testing of the autodiff-valued contact
 surface. It computes the intersection between a rigid triangle mesh (a single
 large triangle) and simple tetrahedral mesh (single tet).

 We define the triangle to be arbitrarily oriented and positioned in the world
 frame. This is analogous to the plane in the mesh-plane intersection test. The
 triangle is defined to lie on the Rz = 0 plane with its normal pointing in the
 Rz direction (just like the canonical definition of a plane).

 The volume mesh is a single tetrahedron with vertices at (0, 0, 0),
 (1, 0, 0), (0, 1, 0), and (0, 0, 1) in the soft mesh frame S.

              Sz
              ┆   ╱
           v3 ●  ╱
              ┆ ╱
           v0 ┆╱    v2
     ┄┄┄┄┄┄┄┄┄●┄┄┄┄┄┄┄●┄┄┄ Sy
             ╱┆
            ╱ ┆
        v1 ●  ┆
          ╱   ┆
        Sx

 We will create a number of fixed poses of the tet w.r.t. the triangle:

   - horizontal slice:
   - triangle slice:
   - quad slice:

 The function TestPositionDerivative() will pose the tetrahedral mesh and
 compute a contact surface. It invokes a provided functor to assess the reported
 derivatives of some arbitrary quantity of the contact surface with respect to
 the position of the origin of frame S.

 @note: This test is copied-and-pasted wholesale from
 mesh_plane_intersection_test.cc. We've altered it in the following ways:
   - swapped the rigid plane with a rigid triangle mesh,
   - changed the call to the intersecting algorithm, and
   - updated the documentation so that references to "plane" now refer to the
     "triangle" (or "tri" for brevity).
 Otherwise, it's line for line and comment for comment identical. We might want
 to consider refactoring it but for the fact that it only appears two places. If
 we end up supporting other customized implementations between planar rigid
 objects and soft mesh, it would make sense to refactor.

 We do all the tests with TriangleSurfaceMesh; the difference between
 TriangleSurfaceMesh and PolygonalSurfaceMesh is inconsequential to this test.
 */
class MeshMeshDerivativesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    id_S_ = GeometryId::get_new_id();
    /* The pressure field is arbitrary, but
      1. its orientation doesn't lead to culling of intersection polygons, and
      2. validation is expressed in terms of that gradient. */
    vector<VolumeElement> elements({VolumeElement(0, 1, 2, 3)});
    vector<Vector3d> vertices_S({Vector3d::Zero(), Vector3d::UnitX(),
                                 Vector3d::UnitY(), Vector3d::UnitZ()});
    tet_mesh_S_ = make_unique<VolumeMesh<double>>(std::move(elements),
                                              std::move(vertices_S));
    field_S_ = make_unique<VolumeMeshFieldLinear<double, double>>(
        vector<double>{0.25, 0.5, 0.75, 1}, tet_mesh_S_.get());
    bvh_S_ = std::make_unique<Bvh<Obb, VolumeMesh<double>>>(*tet_mesh_S_);

    /* Rigid triangle mesh; tilt and offset the triangle's plane so things are
     interesting. */
    vector<Vector3d> vertices{Vector3d{-5, -5, 0},
                              Vector3d{5, -5, 0},
                              Vector3d{0, 5, 0}};
    vector<SurfaceTriangle> faces{{0, 1, 2}};
    tri_mesh_R_ =
        make_unique<TriangleSurfaceMesh<double>>(move(faces), move(vertices));
    bvh_R_ = make_unique<Bvh<Obb, TriangleSurfaceMesh<double>>>(*tri_mesh_R_);
    X_WR_ = HalfSpace::MakePose(Vector3d{1, 2, 3}.normalized(),
                                Vector3d{0.25, 0.1, -0.2})
                .cast<AutoDiffXd>();
    id_R_ = GeometryId::get_new_id();
  }

  /* Creates a rotation to align two vectors: `s` and `t` such that t = Rs.

   Note: If we want to differentiate w.r.t. orientation, it is imperative that
   `s` not point in the `t` or `-t` directions.

   @pre |s| = |t| = 1. */
  static RotationMatrixd OrientTetrahedron(const Vector3d& s_F,
                                           const Vector3d& t_F) {
    // TODO(SeanCurtis-TRI) I stole this code from characterization_utilities.cc
    //  One more re-use and we have the opportunity to refactor.

    const double cos_theta = s_F.dot(t_F);
    constexpr double kAlmostOne = 1 - std::numeric_limits<double>::epsilon();

    /* Default to identity if s and t are already aligned. */
    RotationMatrixd R_FA;
    if (cos_theta < kAlmostOne) {
      /* They aren't already parallel. */
      if (cos_theta < -kAlmostOne) {
        /* They are anti-parallel. We need a normal perpendicular to s_F;
         extract it from a valid basis. */
        const math::RotationMatrix<double> basis =
            math::RotationMatrix<double>::MakeFromOneVector(s_F, 2);
        const Vector3d rhat = basis.col(0);
        R_FA = RotationMatrixd(AngleAxisd(M_PI, rhat));
      } else {
        const Vector3d rhat = s_F.cross(t_F).normalized();
        R_FA = RotationMatrixd(AngleAxisd(acos(cos_theta), rhat));
      }
    }
    return R_FA;
  }

  /* Indicator for the relative pose of the tet relative to the triangle. See
   class documentation for details. */
  enum TetPose { kHorizontalSlice, kTriangleSlice, kQuadSlice };

  /* Tests for an arbitrary quantity of the contact surface against multiple
   relative poses between triangle and tetrahedron. We evaluate three different
   configurations (as documented in the function). For each configuration, we
   invoke evaluate_quantity().

   @param evaluate_quantity  A function that assess some aspect of the contact
                             surface and its derivatives. It must be written
                             to account for the documented relative poses
                             between triangle and mesh. The provided function
                             should make use of googletest EXPECT_* macros to
                             perform the assessment. */
  void TestPositionDerivative(
      const std::function<void(const ContactSurface<AutoDiffXd>&,
                               const RigidTransform<AutoDiffXd>&, TetPose)>
          evaluate_quantity) const {
    struct Configuration {
      std::string name;
      Vector3d p_RS_d;
      RotationMatrixd R_RS_d;
      int num_faces{};
      TetPose pose;
    };
    vector<Configuration> configurations;

    const RigidTransformd X_WR_d = convert_to_double(X_WR_);
    const Vector3d n_R{0, 0, 1};
    // We want to make sure that p_SoRo is completely non-zero. So, we pick
    // a point N on the triangle, offset from its origin and position the tet
    // with respect to that point.
    const Vector3d p_RN{0.25, -0.3, 0};
    const Vector3d p_RS_d(p_RN - kDepth * n_R);

    {
      /* Leave the tetrahedron unrotated, i.e. R_RS = I. The contact mesh will
       be a "horizontal" slice of the tet: a right isosceles triangle. */
      configurations.push_back(
          {"Horizontal slice", p_RS_d, RotationMatrixd{}, 3, kHorizontalSlice});
    }

    {
      /* Vertex 0 (<0, 0, 0> in S) lies kDepth units on the negative side of the
       triangle, all other vertices lie on the other side. The vector
       Sx + Sy + Sz = <1, 1, 1> points out of vertex 0 and into the opposing
       tetrahedral face. We'll align it with the triangle normal. The contact
       surface will be an equilateral triangle. */
      const RotationMatrixd R_RS_d =
          OrientTetrahedron(Vector3d{1, 1, 1}.normalized(), n_R);
      configurations.push_back(
          {"Single vertex penetration", p_RS_d, R_RS_d, 3, kTriangleSlice});
    }

    {
      /* Vertices 0 and 3 (<0, 0, 1>) lie kDepth units on the negative side of
       the triangle, vertices 1 & 2 on the positive side. The vector
       Sx + Sy = <1, 1, 0> is perpendicular to the edge spanned by V0 and V3
       (pointing into the tetrahedron). We'll align that with the triangle
       normal. This should leave the edges (0, 3) and (1, 2) both parallel with
       the triangle's plane. */
      const RotationMatrixd R_RS_d =
          OrientTetrahedron(Vector3d{1, 1, 0}.normalized(), n_R);

      /* Reality check: confirm the edges *are* parallel with the triangle. */
      const RotationMatrixd R_WR_d = convert_to_double(X_WR_).rotation();
      const RotationMatrixd R_WS_d = R_WR_d * R_RS_d;
      const Vector3d& v0_S = tet_mesh_S_->vertex(0);
      const Vector3d& v1_S = tet_mesh_S_->vertex(1);
      const Vector3d& v2_S = tet_mesh_S_->vertex(2);
      const Vector3d& v3_S = tet_mesh_S_->vertex(3);
      const Vector3d e03_S = v3_S - v0_S;
      const Vector3d e12_S = v2_S - v1_S;
      const Vector3d e03_W = R_WS_d * e03_S;
      const Vector3d e12_W = R_WS_d * e12_S;
      const Vector3d n_W_d = R_WR_d * n_R;
      EXPECT_NEAR(n_W_d.dot(e03_W), 0, 1e-15);
      EXPECT_NEAR(n_W_d.dot(e12_W), 0, 1e-15);
      configurations.push_back(
          {"Two vertex penetration", p_RS_d, R_RS_d, 4, kQuadSlice});
    }

    for (const auto& config : configurations) {
      const RotationMatrixd R_WS_d = X_WR_d.rotation() * config.R_RS_d;
      const Vector3d p_WS_d = X_WR_d * config.p_RS_d;
      const Vector3<AutoDiffXd> p_WS = math::InitializeAutoDiff(p_WS_d);
      const RigidTransform<AutoDiffXd> X_WS(R_WS_d.cast<AutoDiffXd>(), p_WS);

      auto surface = ComputeContactSurfaceFromSoftVolumeRigidSurface(
          id_S_, *field_S_, *bvh_S_, X_WS, id_R_, *tri_mesh_R_, *bvh_R_,
          X_WR_, HydroelasticContactRepresentation::kTriangle);

      SCOPED_TRACE(config.name);
      ASSERT_NE(surface, nullptr);
      ASSERT_EQ(surface->tri_mesh_W().num_triangles(), config.num_faces);

      evaluate_quantity(*surface, X_WS, config.pose);
    }
  }

  /* Given the point E which purports to lie on an edge of the tetrahedral mesh,
   finds the edge it lies on (spanning vertices A and B) and returns p_AB_S. */
  Vector3d GetEdgeDirInS(const Vector3d& p_SE) const {
    // We determine the edge that E lies on with this simple metric. If E lies
    // on edge AB, then AB⋅AE = |AB|⋅|AE|, or, to avoid square roots:
    // (AB⋅AE)² = |AB|²⋅|AE|².
    // This wouldn't be sufficient generally. But for this test we can make
    // two simplifying assumptions:
    //   1. E actually does lie on *one* of the mesh edges.
    //   2. The edges are easily distinguishable by direction.
    const vector<Vector3d>& verts_S = field_S_->mesh().vertices();
    const vector<pair<int, int>> edges{{0, 1}, {0, 2}, {0, 3},
                                       {1, 2}, {1, 3}, {2, 3}};
    for (const auto& [a, b] : edges) {
      const Vector3d p_AB_S = verts_S[b] - verts_S[a];
      const Vector3d p_AE_S = p_SE - verts_S[a];
      const double lhs = std::pow(p_AB_S.dot(p_AE_S), 2);
      const double rhs = p_AB_S.squaredNorm() * p_AE_S.squaredNorm();
      if (std::abs(lhs - rhs) < 1e-15) {
        return p_AB_S;
      }
    }
    throw std::logic_error(
        "Querying for point E that isn't actually on a tet edge");
  }

  /* Soft volume mesh. */
  GeometryId id_S_;
  unique_ptr<VolumeMesh<double>> tet_mesh_S_;
  unique_ptr<VolumeMeshFieldLinear<double, double>> field_S_;
  unique_ptr<Bvh<Obb, VolumeMesh<double>>> bvh_S_;

  /* Rigid triangle mesh. */
  RigidTransform<AutoDiffXd> X_WR_;
  unique_ptr<TriangleSurfaceMesh<double>> tri_mesh_R_;
  unique_ptr<Bvh<Obb, TriangleSurfaceMesh<double>>> bvh_R_;
  GeometryId id_R_;

  /* The amount I penetrate triangle into the tet.  */
  static constexpr double kDepth = 0.25;
};

TEST_F(MeshMeshDerivativesTest, Area) {
  /* We'll compute the expected contact surface area (and its derivatives) by
   decomposing it into triangles. For a triangle defined by vertices A, B, and
   C, with triangle normal n̂ (and assuming that the triangle winding is
   consistent with the normal direction):

      Area = 0.5 * |(B - A) × (C - A)|₂
           = 0.5 * [(B - A) × (C - A)]ᵀn̂
           = 0.5 * [skew_sym(B - A) (C-A)]ᵀn̂

        where, a x b = skew_sym(a) b and

                          │  0 -a3  a2│
            skew_sym(a) = │ a3   0 -a1│
                          │-a2  a1   0│

    ∂Area          ∂[skew_sym(B - A) (C-A)]ᵀ
    ────── = 0.5 * ─────────────────── n̂
      ∂So                ∂So

                   │                  ∂(B - A)                    ∂(C - A) │ᵀ
           = 0.5 * │ -skew_sym(C - A) ────────  + skew_sym(B - A) ──────── │ n̂
                   │                    ∂So                          ∂So   │

   We are *given* the quantities ∂A/∂So, ∂B/∂So, and ∂C/∂So which have been
   independently validated by the VertexPosition test. */
  auto evaluate_area = [X_WR = convert_to_double(this->X_WR_)](
                           const ContactSurface<AutoDiffXd>& surface,
                           const RigidTransform<AutoDiffXd>& X_WS_ad,
                           TetPose pose) {
    const auto& mesh_W = surface.tri_mesh_W();

    // For v × A, this makes a matrix V such that VA = v × A.
    auto skew_matrix = [](const Vector3d& v) {
      Matrix3<double> result;
      // clang-format off
      result <<     0, -v(2),  v(1),
                 v(2),     0, -v(0),
                -v(1),  v(0),   0;
      // clang-format on
      return result;
    };

    /* We don't want to compute the areas of the *individual* triangles in the
     contact surface because it is triangle fan around a centroid. The fan and
     centroid contribute nothing to the area. So, as an independent witness,
     we'll compute the area of the *polygon* based on the vertices on the
     perimeter. The triangles defined below are based on a priori knowledge
     of the behavior of the mesh-mesh intersection algorithm. If it changes,
     these hard-coded indices could become invalid. A more robust solution would
     be to *infer* the boundary polygon edges from the contact surface, but
     that's a lot of effort for little present value. */
    vector<vector<int>> triangles;
    switch (pose) {
      case TetPose::kHorizontalSlice:
      case TetPose::kTriangleSlice:
        triangles.emplace_back(vector<int>{0, 1, 2});
        break;
      case TetPose::kQuadSlice:
        /* This is a bit brittle and is predicated on knowledge of how
         the intersection algorithm processes the particular geometry. If that
         proves to be too brittle, we'll need to reconstruct this by looking
         at the provided mesh. */
        triangles.emplace_back(vector<int>{0, 1, 2});
        triangles.emplace_back(vector<int>{2, 3, 1});
        break;
    }

    /* The normal for the contact surface is simply the tri normal: Rz (here,
     expressed in world). */
    const Vector3d n_W = X_WR.rotation().col(2);

    double area_expected = 0;
    Vector3d dArea_dSo_expected = Vector3d::Zero();
    for (const auto& tri : triangles) {
      const auto& p_WA_ad = mesh_W.vertex(tri[0]);
      const auto& p_WB_ad = mesh_W.vertex(tri[1]);
      const auto& p_WC_ad = mesh_W.vertex(tri[2]);
      const Vector3d p_WA = convert_to_double(p_WA_ad);
      const Vector3d p_WB = convert_to_double(p_WB_ad);
      const Vector3d p_WC = convert_to_double(p_WC_ad);
      const Matrix3<double> dA_dSo = math::ExtractGradient(p_WA_ad);
      const Matrix3<double> dB_dSo = math::ExtractGradient(p_WB_ad);
      const Matrix3<double> dC_dSo = math::ExtractGradient(p_WC_ad);

      const Vector3d p_AB_W = p_WB - p_WA;
      const Vector3d p_AC_W = p_WC - p_WA;
      const double tri_area = 0.5 * p_AB_W.cross(p_AC_W).dot(n_W);
      area_expected += tri_area;

      const Matrix3<double> left = -skew_matrix(p_AC_W) * (dB_dSo - dA_dSo);
      const Matrix3<double> right = skew_matrix(p_AB_W) * (dC_dSo - dA_dSo);
      dArea_dSo_expected += 0.5 * ((left + right).transpose() * n_W);
    }

    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const AutoDiffXd& total_area = mesh_W.total_area();
    EXPECT_NEAR(total_area.value(), area_expected, 4 * kEps);
    EXPECT_TRUE(CompareMatrices(total_area.derivatives(), dArea_dSo_expected,
                                4 * kEps));
  };

  TestPositionDerivative(evaluate_area);
}

TEST_F(MeshMeshDerivativesTest, VertexPosition) {
  /* The vertices of the contact surface *always* lie on the tri. Some of the
   vertices come from intersecting tet edges with the tri. The remaining are
   centroids of polygons formed by the intersection. We'll deal with those
   vertices differently.

   - For vertices at the intersection of tri and tet edge, the derivative of
     the vertex position w.r.t. the position of the tetrahedron origin (So) is
     (expressed in the rigid triangle's frame R):

                         | 1  0  -p.x |
          ∂p_RV/∂p_RSo = | 0  1  -p.y |
                         | 0  0   0   |

     In the rigid triangle's frame, the vertices can never move *off* the
     triangle, so ∂V.z/∂So_R must always be zero. ∂V.x/∂So_R and ∂V.y/∂So_R are
     100% coupled with the movement of So in the Rx and Ry directions,
     respectively. So's movement in the Rz direction affects vertex position in
     the Rx and Ry directions based on the angle between the edge and the
     triangle's normal. The intersecting edge e has a component parallel to the
     triangle and a component perpendicular to the triangle. We define the
     vector p as the ratio of movement on the triangle versus motion
     perpendicular to the triangle:

        p = (e − (e⋅n̂)n̂)/(e⋅n̂)

     However, the derivatives reported in the test are ∂p_WV/∂p_WSo, so we have
     to transform the expected result from the rigid triangle frame to world.

          ∂p_WV    ∂(R_WR⋅p_RV + p_WRo)
         ------- = --------------------          // Expand p_WV = X_WR * p_RV.
          ∂p_WSo         ∂p_WSo

                   ∂(R_WR⋅p_RV)
                 = -------------                 // p_WRo doesn't depend on So.
                      ∂p_WSo

                   ∂(R_WR⋅p_RV)     ∂p_RSo
                 = ------------- * --------      // Chain rule.
                      ∂p_RSo        ∂p_WSo

                   ∂(R_WR⋅p_RV)
                 = ------------- * R_RW          // Change of So in R is related
                      ∂p_RSo                     // to change in W by R_RW.

                           ∂p_RV
                 = R_WR * -------- * R_RW        // R_WR doesn't depend on So.
                           ∂p_RSo

                          | 1 0 -p.x |
                 = R_WR * | 0 1 -p.y | * R_RW    // Definition of ∂p_RV/∂p_RSo.
                          | 0 0  0   |

   - We treat centroid vertices differently. For centroids of triangles (where
     the intersecting polygon between tet and triangle is itself a triangle),
     the centroid position and its derivative are simply:

                 C = (v0 + v1 + v2) / 3, and
            ∂C/∂So = (∂v0/∂So + ∂v1/∂So + ∂v2/∂So) / 3

     We skip the centroid for polygons with four or more vertices. Those
     centroids are computed by decomposing the polygon into triangles. For each
     triangle we compute centroid and area and then define the polygon centroid
     as a weighted average of the triangle centroids. There is no simple way to
     validate the derivatives of this vertex, so we'll skip it for now.

   Finally, this test exploits special knowledge that when a polygon with N
   vertices is produced by intersection, N + 1 vertices are added to the contact
   surface mesh: the polygon's N vertices followed by the centroid. In this
   test, we can use that to implicitly recognize which vertices come from edge
   intersections and which are centroids (combined with the fact that there's
   only a single polygon in the contact surface mesh). */
  auto evalute_position = [this](const ContactSurface<AutoDiffXd>& surface,
                                 const RigidTransform<AutoDiffXd>& X_WS_ad,
                                 TetPose pose) {
    constexpr double kEps = 5 * std::numeric_limits<double>::epsilon();

    /* The test is set up so there is only ever a single intersecting polygon.
     So, there is *one* centroid (the last vertex). All other vertices come
     from intersecting a tet edge with the triangle. We'll evaluate all of those
     and then handle the centroid specially. */
    const Vector3d n_R{0, 0, 1};
    const TriangleSurfaceMesh<AutoDiffXd>& mesh_W = surface.tri_mesh_W();
    const RotationMatrixd R_WR = convert_to_double(this->X_WR_).rotation();
    const RotationMatrixd R_RW = R_WR.inverse();
    const RigidTransformd X_WS = convert_to_double(X_WS_ad);
    const RotationMatrixd& R_WS = X_WS.rotation();
    for (int v = 0; v < mesh_W.num_vertices() - 1; ++v) {
      const Vector3<AutoDiffXd>& p_WV_ad = mesh_W.vertex(v);
      const Vector3d& p_WV = convert_to_double(p_WV_ad);
      const Vector3d e_R = R_RW * R_WS * GetEdgeDirInS(X_WS.inverse() * p_WV);
      const double in_normal_dir = e_R.dot(n_R);
      // Reality check: We don't have edges lying on the triangle.
      DRAKE_DEMAND(std::abs(in_normal_dir) > 1e-10);
      const Vector3d p = (e_R - in_normal_dir * n_R) / in_normal_dir;
      Matrix3<double> expected_J_R;
      // clang-format off
      expected_J_R << 1,  0, -p.x(),
                      0,  1, -p.y(),
                      0,  0,  0;
      // clang-format on
      const Matrix3<double> expected_J_W =
          R_WR * (expected_J_R * R_RW.matrix());
      const Matrix3<double> J_W = math::ExtractGradient(p_WV_ad);
      ASSERT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
    }

    /* Now handle the centroid. */
    switch (pose) {
      case kHorizontalSlice:
      case kTriangleSlice: {
        /* The derivative should simply be the mean of the first three. */
        Matrix3<double> expected_J_W = Matrix3<double>::Zero();
        for (int v = 0; v < 3; ++v) {
          expected_J_W += math::ExtractGradient(mesh_W.vertex(v));
        }
        expected_J_W /= 3;
        const Vector3<AutoDiffXd>& p_WC = mesh_W.vertex(3);
        const Matrix3<double> J_W = math::ExtractGradient(p_WC);
        EXPECT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
        break;
      }
      case kQuadSlice:
        /* We skip the centroid for the quad case. */
        break;
    }
  };

  TestPositionDerivative(evalute_position);
}

TEST_F(MeshMeshDerivativesTest, FaceNormalsWrtPosition) {
  /* Face normals should always be parallel with the triangle normal. */
  auto evaluate_normals = [X_WR = convert_to_double(this->X_WR_)](
                              const ContactSurface<AutoDiffXd>& surface,
                              const RigidTransform<AutoDiffXd>& X_WS, TetPose) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const auto& mesh_W = surface.tri_mesh_W();
    const Vector3d plane_n_W = X_WR.rotation().col(2);
    const Matrix3<double> zeros = Matrix3<double>::Zero();
    for (int f = 0; f < mesh_W.num_elements(); ++f) {
      const Vector3<AutoDiffXd>& tri_n_W = mesh_W.face_normal(f);
      EXPECT_TRUE(
          CompareMatrices(math::ExtractValue(tri_n_W), plane_n_W, 2 * kEps));
      EXPECT_TRUE(
          CompareMatrices(math::ExtractGradient(tri_n_W), zeros, 10 * kEps));
    }
  };

  TestPositionDerivative(evaluate_normals);
}

TEST_F(MeshMeshDerivativesTest, FaceNormalsWrtOrientation) {
  /* Even if the soft frame is rotated w.r.t. the rigid frame, the contact
   surface normals should always be parallel with the triangle's normal and,
   therefore, should have zero derivatives.

   This test does *not* use the TestPositionDerivative() API because that
   differentiates with respect to p_WSo and makes assumptions about the
   resulting mesh. For this test, we need a different derivative and different
   assumptions, so we'll simply duplicate that portion of
   TestPositionDerivative() that is relevant for this test. */

  /* For simplicity, we'll simply differentiate w.r.t. a single scalar: a
   rotation of θ radians around an arbitrary axis. We'll sample a few values,
   but make sure that the mesh normal and field gradient remain sufficiently
   aligned that we don't lose the triangle due to "backface culling".  */
  const Vector3d v_W = Vector3d{-1, 2, -3}.normalized();
  // Rather than aligning the tet mesh with the *origin* of the rigid frame,
  // we pick some point (N) away from the origin, but still on the triangle.
  const Vector3<AutoDiffXd> p_WN =
      this->X_WR_ * Vector3<AutoDiffXd>{0.25, -0.3, 0};
  const Vector3<AutoDiffXd> plane_n_W_ad = this->X_WR_.rotation().col(2);
  const Vector3d plane_n_W = math::ExtractValue(plane_n_W_ad);
  const Vector3<AutoDiffXd> p_WS = p_WN - this->kDepth * plane_n_W_ad;
  for (const double theta : {0.0, M_PI / 6, M_PI / 2 * 0.9, M_PI / 2 * 0.99}) {
    AutoDiffXd theta_ad = theta;
    theta_ad.derivatives().resize(1);
    theta_ad.derivatives() << 1;
    RigidTransform<AutoDiffXd> X_WS{
        RotationMatrix<AutoDiffXd>(AngleAxis<AutoDiffXd>(theta_ad, v_W)), p_WS};

    auto surface = ComputeContactSurfaceFromSoftVolumeRigidSurface(
        this->id_S_, *this->field_S_, *this->bvh_S_, X_WS, this->id_R_,
        *this->tri_mesh_R_, *this->bvh_R_, this->X_WR_,
        HydroelasticContactRepresentation::kTriangle);

    SCOPED_TRACE(fmt::format("theta = {:.5f} radians", theta));
    ASSERT_NE(surface, nullptr);
    /* Make sure the test doesn't pass simply because we have no triangles. */
    const auto& mesh_W = surface->tri_mesh_W();
    ASSERT_GT(mesh_W.num_elements(), 0);

    constexpr double kEps = std::numeric_limits<double>::epsilon();
    for (int f = 0; f < mesh_W.num_elements(); ++f) {
      const Vector3<AutoDiffXd>& tri_n_W = mesh_W.face_normal(f);
      /* Confirm the normal direction. */
      EXPECT_TRUE(
          CompareMatrices(math::ExtractValue(tri_n_W), plane_n_W, 5 * kEps));
      /* Confirm the normal gradient w.r.t. theta. */
      EXPECT_TRUE(CompareMatrices(math::ExtractGradient(tri_n_W),
                                  Vector3d::Zero(), 10 * kEps));
    }
  }
}

TEST_F(MeshMeshDerivativesTest, Pressure) {
  /* Because the field is a linear field, we can think of the pressure function,
   evaluated at point Q as p(Q) = ∇pᵀ(Q - E), such that ∇p is the gradient of
   the pressure field and E is a point at which the pressure field is zero.
   We'll define p_W(p_WQ) = ∇p_Wᵀ(p_WQ - p_WE) as the pressure evaluated on
   points measured and expressed in the world frame. So,

                    ∂[∇p_Wᵀ(p_WQ - p_WE)]
      ∂p_WQ/∂p_WSo = ─────────────────────
                           ∂p_WSo

                            ∂[(p_WQ - p_WE)]
                   = ∇p_Wᵀ ──────────────────      // ∇p_W const w.r.t. p_WSo.
                                 ∂p_WSo

                           ┌                   ┐
                           │ ∂p_WQ      ∂p_WE  │
                   = ∇p_Wᵀ │──────── - ────────│   // Transpose and subtraction
                           │ ∂p_WSo     ∂p_WSo │   // are is linear operators.
                           └                   ┘

                           ┌                  ┐
                           │ ∂p_WQ     │1 0 0││
                   = ∇p_Wᵀ │──────── - │0 1 0││    // E affixed to Frame S.
                           │ ∂p_WSo    │0 0 1││
                           └                  ┘

   ∂p_WQ/∂p_WSo are the derivatives that were confirmed in the VertexPosition
   test, so we can use those to compute the expected pressure derivative. */
  const Vector3d grad_p_S = field_S_->EvaluateGradient(0);
  auto evaluate_pressure = [X_WR = convert_to_double(this->X_WR_), &grad_p_S](
                               const ContactSurface<AutoDiffXd>& surface,
                               const RigidTransform<AutoDiffXd>& X_WS,
                               TetPose) {
    constexpr double kEps = 8 * std::numeric_limits<double>::epsilon();
    const RigidTransform<double> X_WS_d = convert_to_double(X_WS);
    const Vector3d grad_p_W = X_WS_d.rotation() * grad_p_S;
    for (int v = 0; v < surface.tri_mesh_W().num_vertices(); ++v) {
      const Matrix3<double> dp_WQ_dp_WSo_W =
          math::ExtractGradient(surface.tri_mesh_W().vertex(v));
      const Vector3d dp_dp_WSo_W_expected =
          grad_p_W.transpose() * (dp_WQ_dp_WSo_W - Matrix3<double>::Identity());
      const AutoDiffXd& p = surface.tri_e_MN().EvaluateAtVertex(v);
      EXPECT_TRUE(CompareMatrices(p.derivatives(), dp_dp_WSo_W_expected, kEps));
    }
  };

  TestPositionDerivative(evaluate_pressure);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
