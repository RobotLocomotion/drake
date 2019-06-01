#include "drake/geometry/proximity/mesh_intersection.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransform;
using math::RollPitchYaw;

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
      return (u - v).norm() < std::numeric_limits<double>::epsilon();
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

// TODO(DamrongGuoy): Add unit tests for CalcIntersection().

// TODO(DamrongGuoy): Add unit tests for ClipPolygonByHalfspace().

GTEST_TEST(MeshIntersectionTest, ClipTriangleByTetrahedron) {
  auto volume_M = TrivialVolumeMesh<double>();
  auto surface_N = TrivialSurfaceMesh<double>();
  const VolumeElementIndex element0(0);
  const VolumeElementIndex element1(1);
  SurfaceFaceIndex face(0);

  // The triangle in surface_N coincides with the shared face between the two
  // tetrahedral elements of volume_M.
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
    // Expect "double count". Both tetrahedral elements give the same
    // intersecting polygon, which is a triangle.
    // TODO(DamrongGuoy): Change the expectation when we solve the "double
    //  count" problem.
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

// TODO(DamrongGuoy): More comprehensive tests.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceSoftRigid) {
  auto id_M = GeometryId::get_new_id();
  auto id_N = GeometryId::get_new_id();
  auto soft_mesh_M = OctahedronVolume<double>();
  auto soft_M = OctahedronPressureField<double>(soft_mesh_M.get());
  auto rigid_N = PyramidSurface<double>();
  // Move the rigid pyramid up, so only its square base intersects the top
  // part of the soft octahedron.
  const auto X_MN = math::RigidTransform<double>(Vector3<double>(0, 0, 0.5));

  auto contact_MN_M = mesh_intersection::ComputeContactSurfaceSoftRigid(
      id_M, id_N, *soft_M, *rigid_N, X_MN);
  EXPECT_EQ(12, contact_MN_M->mesh().num_faces());
  const SurfaceFaceIndex face0(0);
  const SurfaceMesh<double>::Barycentric centroid(1./3., 1./3., 1./3.);
  const auto grad_h_M = contact_MN_M->EvaluateGrad_h_MN_M(face0, centroid);
  EXPECT_TRUE(grad_h_M(2) < 0.);
}

// TODO(DamrongGuoy): More comprehensive tests.
GTEST_TEST(MeshIntersectionTest, ComputeContactSurfaceRigidSoft) {
  auto id_A = GeometryId::get_new_id();
  auto id_B = GeometryId::get_new_id();
  auto rigid_A = PyramidSurface<double>();
  auto soft_mesh_B = OctahedronVolume<double>();
  auto soft_B = OctahedronPressureField<double>(soft_mesh_B.get());
  // Move the soft octahedron down, so only its top part intersects the
  // square base of the rigid pyramid.
  const auto X_AB = math::RigidTransform<double>(Vector3<double>(0, 0, -0.5));

  auto contact_AB_A = mesh_intersection::ComputeContactSurfaceRigidSoft(
      id_A, id_B, *rigid_A, *soft_B, X_AB);
  EXPECT_EQ(12, contact_AB_A->mesh().num_faces());
  const SurfaceFaceIndex face0(0);
  const SurfaceMesh<double>::Barycentric centroid(1./3., 1./3., 1./3.);
  const auto grad_h_M = contact_AB_A->EvaluateGrad_h_MN_M(face0, centroid);
  EXPECT_TRUE(grad_h_M(2) > 0.);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
