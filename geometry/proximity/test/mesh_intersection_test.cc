#include "drake/geometry/proximity/mesh_intersection.h"

#include <algorithm>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransform;
using math::RollPitchYaw;

// TODO(DamrongGuoy): More comprehensive tests.
GTEST_TEST(MeshIntersectionTest, CalcIntersection) {
  const double kEps = std::numeric_limits<double>::epsilon();
  // Halfspace {(x,y,z) : x <= 2.0}
  const Vector3<double> unit_normal = Vector3<double>::UnitX();
  const double plane_offset = 2.0;
  const fcl::Halfspace<double> halfspace(unit_normal, plane_offset);

  // The line AB intersects the plane of the halfspace.
  {
    const Vector3<double> A = Vector3<double>::Zero();
    const Vector3<double> B(4, 6, 10);
    const Vector3<double> intersection =
        mesh_intersection::CalcIntersection(A, B, halfspace);
    const Vector3<double> expect_intersection(2, 3, 5);
    EXPECT_LE((expect_intersection - intersection).norm(), kEps);
  }

  // The line AB is almost parallel to the plane of the halfspace.
  {
    const Vector3<double> A(plane_offset + 2.0 * kEps, 0., 0.);
    const Vector3<double> B(plane_offset - 2.0 * kEps, 1., 1.);
    const Vector3<double> intersection =
        mesh_intersection::CalcIntersection(A, B, halfspace);
    const Vector3<double> expect_intersection(2., 0.5, 0.5);
    EXPECT_LE((expect_intersection - intersection).norm(), kEps);
  }
}

// TODO(DamrongGuoy): Move the definition of this function here after 11612
//  landed.
template <typename T>
bool CompareConvexPolygon(const std::vector<Vector3<T>>& polygon0,
                          const std::vector<Vector3<T>>& polygon1);

GTEST_TEST(MeshIntersectionTest, ClipPolygonByHalfspace) {
  // Halfspace {(x,y,z) : x <= 2.0}
  const Vector3<double> unit_normal = Vector3<double>::UnitX();
  const double offset = 2.0;
  const fcl::Halfspace<double> halfspace(unit_normal, offset);

  // The input polygon is on the plane X=0, which is parallel to the plane of
  // the halfspace and is completely inside the halfspace. Expect the input
  // polygon and the output polygon to be the same.
  {
    const std::vector<Vector3<double>> input_polygon{
        {0., 0., 0.},
        {0., 1., 0.},
        {0., 1., 1.},
        {0., 0., 1.}
    };
    const std::vector<Vector3<double>> output_polygon =
        mesh_intersection::ClipPolygonByHalfspace(input_polygon, halfspace);
    EXPECT_TRUE(CompareConvexPolygon(input_polygon, output_polygon));
  }
  // The input polygon is on the plane X=3, which is parallel to the plane of
  // the halfspace and is completely outside the halfspace. Expect the output
  // polygon to be empty.
  {
    const std::vector<Vector3<double>> input_polygon{
        {3., 0., 0.},
        {3., 1., 0.},
        {3., 1., 1.},
        {3., 0., 1.}
    };
    const std::vector<Vector3<double>> output_polygon =
        mesh_intersection::ClipPolygonByHalfspace(input_polygon, halfspace);
    const std::vector<Vector3<double>> expect_empty_polygon;
    EXPECT_TRUE(CompareConvexPolygon(expect_empty_polygon, output_polygon));
  }
  // The input polygon is on the plane X=2 of the halfspace. Expect the input
  // polygon and the output polygon to be the same.
  {
    const std::vector<Vector3<double>> input_polygon{
        {2., 0., 0.},
        {2., 1., 0.},
        {2., 1., 1.},
        {2., 0., 1.}
    };
    const std::vector<Vector3<double>> output_polygon =
        mesh_intersection::ClipPolygonByHalfspace(input_polygon, halfspace);
    EXPECT_TRUE(CompareConvexPolygon(input_polygon, output_polygon));
  }
  // The input polygon is outside the halfspace, but it has one edge on the
  // plane of the halfspace. Expect the output polygon to be a zero-area
  // rectangle with two pairs of duplicated vertices.
  {
    const std::vector<Vector3<double>> input_polygon{
        {2., 0., 0.},
        {2., 2., 0.},
        {3., 2., 0.},
        {3., 0., 0.}
    };
    const std::vector<Vector3<double>> output_polygon =
        mesh_intersection::ClipPolygonByHalfspace(input_polygon, halfspace);
    const std::vector<Vector3<double>> expect_output_polygon{
        {2., 0., 0.},
        {2., 0., 0.},
        {2., 2., 0.},
        {2., 2., 0.},
    };
    EXPECT_TRUE(CompareConvexPolygon(expect_output_polygon, output_polygon));
  }
  // The input polygon is outside the halfspace, but it has one vertex on the
  // plane of the halfspace. Expect the output polygon to be a zero-area
  // triangle with three duplicated vertices.
  {
    const std::vector<Vector3<double>> input_polygon{
        {2., 0., 0.},
        {3., 2., 0.},
        {3., 0., 0.}
    };
    const std::vector<Vector3<double>> output_polygon =
        mesh_intersection::ClipPolygonByHalfspace(input_polygon, halfspace);
    const std::vector<Vector3<double>> expect_output_polygon{
        {2., 0., 0.},
        {2., 0., 0.},
        {2., 0., 0.}
    };
    EXPECT_TRUE(CompareConvexPolygon(expect_output_polygon, output_polygon));
  }
}


// Generates a trivial surface mesh consisting of one triangle with vertices
// at the origin and on the X- and Y-axes. We will use it for testing
// triangle-tetrahedron intersection.
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
template <typename T>
std::unique_ptr<SurfaceMesh<T>> TrivialSurfaceMesh() {
  const int face_data[3] = {0, 1, 2};
  std::vector<SurfaceFace> faces {SurfaceFace(face_data)};
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

template <typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TrivialVolumeMeshField(
    VolumeMesh<T>* volume_mesh) {
  // Pressure field value pᵢ at vertex vᵢ.
  const T p0{0.};
  const T p1{0.};
  const T p2{0.};
  const T p3{1.};
  const T p4{1.};
  std::vector<T> p_values = {p0, p1, p2, p3, p4};
  auto volume_mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "pressure", std::move(p_values), volume_mesh);

  return volume_mesh_field;
}

// Checks whether two convex polygons are equal. Here, equality means they
// have the same set of vertices in the same cyclic order.
template <typename T>
bool CompareConvexPolygon(const std::vector<Vector3<T>>& polygon0,
                          const std::vector<Vector3<T>>& polygon1) {
  // Check that they have the same number of vertices.
  if (polygon0.size() != polygon1.size()) return false;
  // Two null polygons are considered equal.
  if (polygon0.size() == 0) return true;
  // Two vertices are the same if they are within epsilon of the other.
  struct Same {
    explicit Same(const Vector3<T>& u_in) : u(u_in) {}
    bool operator()(const Vector3<T>& v) {
      // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
      // too small.
      const T kEps(1e-14);
      return (u - v).norm() < kEps;
    }
   private:
    const Vector3<T>& u;
  };
  auto it = std::find_if(polygon1.begin(), polygon1.end(), Same(polygon0[0]));
  if (it == polygon1.end()) {
    return false;
  }
  int i1 = it - polygon1.begin();
  for (int i0 = 0; size_t(i0) < polygon0.size(); ++i0) {
    if (!Same(polygon0[i0])(polygon1[i1])) return false;
    i1 = (i1 + 1) % polygon1.size();
  }
  return true;
}

GTEST_TEST(MeshIntersectionTest, ClipTriangleByTetrahedron) {
  auto volume_M = TrivialVolumeMesh<double>();
  auto surface_N = TrivialSurfaceMesh<double>();
  const VolumeElementIndex element0(0);
  const VolumeElementIndex element1(1);
  SurfaceFaceIndex face(0);

  // The triangle is outside the first tetrahedron with one vertex on a face
  // of the tetrahedron. Expect the output polygon to be empty.
  {
    const auto X_MN = RigidTransform<double>(Vector3<double>::UnitX());
    const auto polygon = mesh_intersection::ClipTriangleByTetrahedron(
        element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3<double>> expect_empty_polygon;
    EXPECT_TRUE(CompareConvexPolygon(expect_empty_polygon, polygon));
  }
  // The triangle is outside the tetrahedron `element0` with one edge on a
  // face of the tetrahedron. Expect the output polygon to be empty.
  {
    const auto X_MN = RigidTransform<double>(RollPitchYaw<double>(0, 0, M_PI_2),
                                             Vector3<double>::Zero());
    const auto polygon = mesh_intersection::ClipTriangleByTetrahedron(
        element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3<double>> expect_empty_polygon;
    EXPECT_TRUE(CompareConvexPolygon(expect_empty_polygon, polygon));
  }

  // The triangle in surface_N coincides with the shared face between the two
  // tetrahedral elements of volume_M. Expect "double count". Both
  // tetrahedral elements give the same intersecting polygon, which is the
  // triangle.
  // TODO(DamrongGuoy): Change the expectation when we solve the "double
  //  count" problem.
  {
    const auto X_MN = RigidTransform<double>::Identity();
    const auto polygon0_M = mesh_intersection::ClipTriangleByTetrahedron(
        element0, *volume_M, face, *surface_N, X_MN);
    const auto polygon1_M = mesh_intersection::ClipTriangleByTetrahedron(
        element1, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3<double>> expect_triangle_M{
        Vector3<double>::Zero(),
        Vector3<double>::UnitX(),
        Vector3<double>::UnitY()};
    EXPECT_TRUE(CompareConvexPolygon(expect_triangle_M, polygon0_M));
    EXPECT_TRUE(CompareConvexPolygon(expect_triangle_M, polygon1_M));
  }

  // The triangle in surface_N intersect the first tetrahedron but not the
  // second tetrahedron.
  {
    const auto X_MN = RigidTransform<double>(Vector3<double>(0, 0, 0.5));

    const auto polygon0_M = mesh_intersection::ClipTriangleByTetrahedron(
        element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3<double>> expect_triangle_M{
        {0,   0, 0.5},
        {0.5, 0, 0.5},
        {0, 0.5, 0.5}};
    EXPECT_TRUE(CompareConvexPolygon(expect_triangle_M, polygon0_M));

    const auto polygon1_M = mesh_intersection::ClipTriangleByTetrahedron(
        element1, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3<double>> expect_null_triangle;
    EXPECT_TRUE(CompareConvexPolygon(expect_null_triangle, polygon1_M));
  }

  // The triangle intersects the first tetrahedron in a square.
  {
    const auto X_MN = RigidTransform<double>(RollPitchYaw<double>(0, 0, M_PI),
                                             Vector3<double>(0.5, 0.5, 0));
    const auto polygon0_M = mesh_intersection::ClipTriangleByTetrahedron(
        element0, *volume_M, face, *surface_N, X_MN);
    const std::vector<Vector3<double>> expect_square_M{
        {0,   0,   0},
        {0.5, 0,   0},
        {0.5, 0.5, 0},
        {0,   0.5, 0}};
    EXPECT_TRUE(CompareConvexPolygon(expect_square_M, polygon0_M));
  }
  // TODO(DamrongGuoy): Test other cases like:
  //  - the intersecting polygon is a pentagon,
  //  - the intersecting polygon is a hexagon,
  //  - More general X_MN.
}

// Tests a triangle intersect a tetrahedron into a heptagon (seven-sided
// polygon). Strategy:
// 1. Create a tetrahedron that intersects the X-Y plane (Z=0)into a square.
//    A tetrahedron with vertices v0,v1,v2,v3 in this picture will do:
//
//                   +Z   v1(-2,0,2)
//                    |  /|
//                    | / |
//                    |/  +
//                    +  /
//  v0 is (2,0,2).   /| /
//                  / |/
//           +-----/--+------+----+Y
//           |    v0 /|      |
//           |    | / |      |
//  (0,-2,-2)v3---|/--+------v2(0,2,-2)
//                +
//               /
//              /
//            +X
//
// This tetrahedron intersects the X-Y plane into a square with vertices at
// u0 = (v0+v2)/2, u1 = (v1+v2)/2, u2 = (v1+v3)/2, u3 = (v0+v3)/2,
// u0 = (1,1,0),   u1 = (-1,1,0),  u2 = (-1,-1,0), u3 = (1,-1,0).
//
// In X-Y plane, the square u0,u1,u2,u3 will look like this:
//
//
//             +Y          t0
//              |
//      u1------1-------u0
//      |       |       |
//      |       |       |
//      |       |       |
// t1---+-------+-------1---+X
//      |       |       |
//      |       |       |
//      |       |       |
//      u2------+-------u3
//              |
//              t2
//
// 2. Create a triangle on the X-Y plane that intersects the square into a
//    heptagon. A triangle with vertices t0(1.5,1.5,0), t1(-1.5,0,0), and
//    t2(0,-1.5,0) will do. See the above picture.
//
GTEST_TEST(MeshIntersectionTest, ClipTriangleByTetrahedronIntoHeptagon) {
  std::unique_ptr<VolumeMesh<double>> volume_M;
  {
    const int element_data[4] = {0, 1, 2, 3};
    std::vector<VolumeElement> elements {VolumeElement(element_data)};
    const Vector3<double> vertex_data[4] = {
        2.0 * ( Vector3<double>::UnitX() + Vector3<double>::UnitZ()),
        2.0 * (-Vector3<double>::UnitX() + Vector3<double>::UnitZ()),
        2.0 * ( Vector3<double>::UnitY() - Vector3<double>::UnitZ()),
        2.0 * (-Vector3<double>::UnitY() - Vector3<double>::UnitZ())
    };
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
    std::vector<SurfaceFace> faces {SurfaceFace(face_data)};
    const Vector3<double> vertex_data[3] = {
        { 1.5, 1.5, 0.},
        {-1.5, 0.,  0.},
        { 0., -1.5, 0.}
    };
    std::vector<SurfaceVertex<double>> vertices;
    for (auto& vertex : vertex_data) {
      vertices.emplace_back(vertex);
    }
    surface_N = std::make_unique<SurfaceMesh<double>>(std::move(faces),
                                                      std::move(vertices));
  }
  const VolumeElementIndex tetrahedron(0);
  const SurfaceFaceIndex triangle(0);
  const auto X_MN = RigidTransform<double>::Identity();
  const auto polygon_M = mesh_intersection::ClipTriangleByTetrahedron(
      tetrahedron, *volume_M, triangle, *surface_N, X_MN);
  const std::vector<Vector3<double>> expect_heptagon_M {
      { 1.,    1.,   0.},
      { 0.5,   1.,   0.},
      {-1.,    0.25, 0.},
      {-1.,   -0.5,  0.},
      {-0.5,  -1.,   0.},
      { 0.25, -1.,   0.},
      { 1.,    0.5,  0.}
  };
  EXPECT_EQ(7, polygon_M.size());
  EXPECT_TRUE(CompareConvexPolygon(expect_heptagon_M, polygon_M));
}

// TODO(DamrongGuoy): Add unit tests for AddFacesVertices().

// TODO(DamrongGuoy): Add unit tests for ComputeNormalField().

// TODO(DamrongGuoy): Test IntersectSoftVolumeRigidSurface with more general
//  X_MN.  Right now X_MN is a simple translation without rotation.

GTEST_TEST(MeshIntersectionTest, IntersectSoftVolumeRigidSurface) {
  auto soft_mesh_M = TrivialVolumeMesh<double>();
  auto soft_M = TrivialVolumeMeshField<double>(soft_mesh_M.get());
  auto rigid_N = TrivialSurfaceMesh<double>();
  const auto X_MN = math::RigidTransform<double>(Vector3<double>(0, 0, 0.5));

  std::unique_ptr<SurfaceMesh<double>> surface;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_field;
  std::unique_ptr<SurfaceMeshFieldLinear<Vector3<double>, double>> grad_h_field;
  mesh_intersection::IntersectSoftVolumeRigidSurface(
      *soft_M, *rigid_N, X_MN,
      &surface, &e_field, &grad_h_field);

  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_EQ(3, surface->num_faces());
  // TODO(DamrongGuoy): More comprehensive checks.
  const double area0 = surface->area(SurfaceFaceIndex(0));
  const double area1 = surface->area(SurfaceFaceIndex(1));
  const double area2 = surface->area(SurfaceFaceIndex(2));
  const double expect_area = (1./2.) * 0.5 * 0.5 / 3.0;
  EXPECT_NEAR(expect_area, area0, kEps);
  EXPECT_NEAR(expect_area, area1, kEps);
  EXPECT_NEAR(expect_area, area2, kEps);
  const SurfaceFaceIndex face0(0);
  const SurfaceMesh<double>::Barycentric centroid(1./3., 1./3., 1./3.);
  const double e = e_field->Evaluate(face0, centroid);
  const double expect_e = 0.5;
  EXPECT_NEAR(expect_e, e, kEps);
  const auto grad_h = grad_h_field->Evaluate(face0, centroid);
  const auto expect_grad_h = Vector3<double>::UnitZ();
  EXPECT_NEAR((grad_h - expect_grad_h).norm(), 0., kEps);
}


// Generates a volume mesh of an octahedron comprising of eight tetrahedral
// elements with vertices on the coordinate axes and the origin like this:
//
//                +Z   -X
//                 |   /
//                 v5 v3
//                 | /
//                 |/
//  -Y---v4------v0+------v2---+Y
//                /|
//               / |
//             v1  v6
//             /   |
//           +X    |
//                -Z
//
template <typename T>
std::unique_ptr<VolumeMesh<T>> OctahedronVolume() {
  const int element_data[8][4] = {
      // The top four tetrahedrons share the top vertex v5.
      {0, 1, 2, 5}, {0, 2, 3, 5}, {0, 3, 4, 5}, {0, 4, 1, 5},
      // The bottom four tetraehdrons share the bottom vertex v6.
      {0, 2, 1, 6}, {0, 3, 2, 6}, {0, 4, 3, 6}, {0, 1, 4, 6}
  };
  std::vector<VolumeElement> elements;
  for (const auto& element : element_data) {
    elements.emplace_back(element);
  }
  const Vector3<T> vertex_data[7] = {
      Vector3<T>::Zero(),
      Vector3<T>::UnitX(),
      Vector3<T>::UnitY(),
     -Vector3<T>::UnitX(),
     -Vector3<T>::UnitY(),
      Vector3<T>::UnitZ(),
     -Vector3<T>::UnitZ()};
  std::vector<VolumeVertex<T>> vertices;
  for (const auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return std::make_unique<VolumeMesh<T>>(std::move(elements),
                                         std::move(vertices));
}

template <typename T>
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
//                 v5 v3
//                 | /
//                 |/
//  -Y---v4------v0+------v2---+Y
//                /
//               /
//             v1
//             /
//           +X
//
template <typename T>
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
  const Vector3<T> vertex_data[6] = {
      Vector3<T>::Zero(),
      Vector3<T>::UnitX(),
      Vector3<T>::UnitY(),
     -Vector3<T>::UnitX(),
     -Vector3<T>::UnitY(),
      Vector3<T>::UnitZ()
  };
  std::vector<SurfaceVertex<T>> vertices;
  for (auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return std::make_unique<SurfaceMesh<T>>(std::move(faces),
                                          std::move(vertices));
}

template <typename T>
void TestComputeContactSurfaceSoftRigid() {
  auto id_M = GeometryId::get_new_id();
  auto id_N = GeometryId::get_new_id();
  auto soft_mesh_M = OctahedronVolume<T>();
  auto soft_M = OctahedronPressureField<T>(soft_mesh_M.get());
  auto rigid_N = PyramidSurface<T>();
  // Move the rigid pyramid up, so only its square base intersects the top
  // part of the soft octahedron.
  const auto X_MN = math::RigidTransform<T>(Vector3<T>(0, 0, 0.5));

  auto contact_MN_M = mesh_intersection::ComputeContactSurfaceSoftRigid(
      id_M, id_N, *soft_M, *rigid_N, X_MN);
  EXPECT_EQ(12, contact_MN_M->mesh().num_faces());
  const SurfaceFaceIndex face0(0);
  const typename SurfaceMesh<T>::Barycentric centroid(1./3., 1./3., 1./3.);
  const auto grad_h_M = contact_MN_M->EvaluateGrad_h_MN_M(face0, centroid);
  EXPECT_TRUE(grad_h_M(2) < T(0.));
}

GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidDouble) {
  TestComputeContactSurfaceSoftRigid<double>();
}

// Check that we can compile with AutoDiffXd.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidAutoDiffXd) {
  TestComputeContactSurfaceSoftRigid<AutoDiffXd>();
}

template <typename T>
void TestComputeContactSurfaceRigidSoft() {
  auto id_A = GeometryId::get_new_id();
  auto id_B = GeometryId::get_new_id();
  auto rigid_A = PyramidSurface<T>();
  auto soft_mesh_B = OctahedronVolume<T>();
  auto soft_B = OctahedronPressureField<T>(soft_mesh_B.get());
  // Move the soft octahedron down, so only its top part intersects the
  // square base of the rigid pyramid.
  const auto X_AB = math::RigidTransform<T>(Vector3<T>(0, 0, -0.5));

  auto contact_AB_A = mesh_intersection::ComputeContactSurfaceRigidSoft(
      id_A, id_B, *rigid_A, *soft_B, X_AB);
  EXPECT_EQ(12, contact_AB_A->mesh().num_faces());
  const SurfaceFaceIndex face0(0);
  const typename SurfaceMesh<T>::Barycentric centroid(1./3., 1./3., 1./3.);
  const auto grad_h_M = contact_AB_A->EvaluateGrad_h_MN_M(face0, centroid);
  EXPECT_TRUE(grad_h_M(2) > T(0.));
}

GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceRigidSoftDouble) {
  TestComputeContactSurfaceRigidSoft<double>();
}

// Check that we can compile with AutoDiffXd.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceRigidSoftAutoDiffXd) {
  TestComputeContactSurfaceRigidSoft<AutoDiffXd>();
}

// Utility to find a vertex in a face of a SurfaceMesh of geometry M at a given
// position `p_M`.  It performs exhaustive search and is not suitable for
// production use.
// @param[in] p_M
//     The search position expressed in M's frame.
// @param[in] surface_M
//     The surface mesh with vertex positions expressed in M's frame.
// @param[out] face
//     A face that contains the vertex.
// @param[out] vertex
//     Barycentric coordinates of the vertex in the face.
// @return
//     true if found.
bool findFaceVertex(Vector3<double> p_M, const SurfaceMesh<double>& surface_M,
                    SurfaceFaceIndex* face,
                    SurfaceMesh<double>::Barycentric* vertex) {
  for (SurfaceFaceIndex f(0); f < surface_M.num_faces(); ++f) {
    for (int i = 0; i < 3; ++i) {
      const SurfaceVertexIndex v = surface_M.element(f).vertex(i);
      if (p_M == surface_M.vertex(v).r_MV()) {
          *face = f;
          *vertex = SurfaceMesh<double>::Barycentric::Zero();
          (*vertex)(i) = 1.;
          return true;
      }
    }
  }
  return false;
}

// TODO(DamrongGuoy): More comprehensive tests.
// Tests ComputeContactSurfaceSoftRigid as we move the rigid geometry around.
// Use double as the representative type argument.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigidMoving) {
  auto id_M = GeometryId::get_new_id();
  auto id_N = GeometryId::get_new_id();
  auto soft_mesh_M = OctahedronVolume<double>();
  auto soft_M = OctahedronPressureField<double>(soft_mesh_M.get());
  auto rigid_N = PyramidSurface<double>();

  const double kEps = std::numeric_limits<double>::epsilon();

  // Tests translation. Move the rigid pyramid down, so its apex is at the
  // center of the soft octahedron.  Check the field values at that point.
  {
    const auto X_MN = math::RigidTransform<double>(-Vector3<double>::UnitZ());
    auto contact_MN_M = mesh_intersection::ComputeContactSurfaceSoftRigid(
        id_M, id_N, *soft_M, *rigid_N, X_MN);
    // TODO(DamrongGuoy): More comprehensive checks on the mesh of the contact
    //  surface. Here we only check the number of triangles.
    EXPECT_EQ(12, contact_MN_M->mesh().num_faces());
    SurfaceFaceIndex face;
    SurfaceMesh<double>::Barycentric apex;
    bool found = findFaceVertex(Vector3<double>::Zero(), contact_MN_M->mesh(),
                                &face, &apex);
    ASSERT_TRUE(found);
    const auto e_MN = contact_MN_M->EvaluateE_MN(face, apex);
    EXPECT_NEAR(1.0, e_MN, kEps);
    const auto grad_h_M = contact_MN_M->EvaluateGrad_h_MN_M(face, apex);
    const Vector3<double> expect_grad_h_M = Vector3<double>::UnitZ();
    EXPECT_NEAR((expect_grad_h_M - grad_h_M).norm(), 0., kEps);
  }
  // Tests rotation. First we rotate the rigid pyramid 90 degrees around
  // X-axis, so it will fit the left half of the soft octahedron, instead of
  // the top half of the octahedron.  The pyramid vertices will look like this:
  //
  //                +Z   -X
  //                 |   /
  //                 v2 v3
  //                 | /
  //                 |/
  //  -Y---v5------v0+-----------+Y
  //                /|
  //               / |
  //             v1  v4
  //             /   |
  //           +X    |
  //                -Z
  //
  //
  // To  avoid "double counting" problem, we then translate the pyramid by
  // half its height to the left.  The center of the contact surface will be
  // at (0, -1/2, 0) in the soft octahedron's frame.
  {
    const auto X_MN = math::RigidTransform<double>(
        RollPitchYaw<double>(M_PI / 2., 0., 0.),
       -Vector3<double>::UnitY() / 2.);
    auto contact_MN_M = mesh_intersection::ComputeContactSurfaceSoftRigid(
        id_M, id_N, *soft_M, *rigid_N, X_MN);
    // TODO(DamrongGuoy): More comprehensive checks on the mesh of the contact
    //  surface.  Here we only check the number of triangles.
    EXPECT_EQ(12, contact_MN_M->mesh().num_faces());
    SurfaceFaceIndex face;
    SurfaceMesh<double>::Barycentric center;
    bool found = findFaceVertex(-Vector3<double>::UnitY() / 2.,
                                contact_MN_M->mesh(), &face, &center);
    ASSERT_TRUE(found);
    const auto e_MN = contact_MN_M->EvaluateE_MN(face, center);
    EXPECT_NEAR(0.5, e_MN, kEps);
    const auto grad_h_M = contact_MN_M->EvaluateGrad_h_MN_M(face, center);
    const Vector3<double> expect_grad_h_M = Vector3<double>::UnitY();
    EXPECT_NEAR((expect_grad_h_M - grad_h_M).norm(), 0., kEps);
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
