#include "drake/geometry/proximity/surface_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace {

// Used for testing instantiation of SurfaceMesh and inspecting its components.
template <typename T>
std::unique_ptr<SurfaceMesh<T>> GenerateTwoTriangleMesh() {
  // The surface mesh will consist of four vertices and two faces and will
  // be constructed such that area and geometric centroid are straightforward
  // to check.

  // Create the vertices.
  std::vector<SurfaceVertex<T>> vertices;
  vertices.emplace_back(Vector3<T>(0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<T>(-0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<T>(-0.5, -0.5, -0.5));
  vertices.emplace_back(Vector3<T>(1.0, -1.0, -0.5));

  // Create the two triangles. Note that SurfaceMesh does not specify (or use) a
  // particular winding.
  std::vector<SurfaceFace> faces;
  faces.emplace_back(
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2));
  faces.emplace_back(
      SurfaceVertexIndex(2), SurfaceVertexIndex(3), SurfaceVertexIndex(0));

  return std::make_unique<SurfaceMesh<T>>(
      std::move(faces), std::move(vertices));
}

// Generates an empty mesh.
std::unique_ptr<SurfaceMesh<double>> GenerateEmptyMesh() {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;
  return std::make_unique<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
}

// Generates a zero-area mesh.
std::unique_ptr<SurfaceMesh<double>> GenerateZeroAreaMesh() {
  // The surface mesh will consist of four vertices and two faces.

  // Create the vertices.
  std::vector<SurfaceVertex<double>> vertices;
  for (int i = 0; i < 4; ++i)
    vertices.emplace_back(Vector3<double>::Zero());

  // Create the two triangles.
  std::vector<SurfaceFace> faces;
  faces.emplace_back(
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2));
  faces.emplace_back(
      SurfaceVertexIndex(2), SurfaceVertexIndex(3), SurfaceVertexIndex(0));

  return std::make_unique<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
}

// Test instantiation of SurfaceMesh of a surface M and inspecting its
// components. By default, the vertex positions are expressed in M's frame.
// The optional parameter X_WM will change the vertex positions to W's frame.
template <typename T>
std::unique_ptr<SurfaceMesh<T>> TestSurfaceMesh(
    const math::RigidTransform<T> X_WM = math::RigidTransform<T>::Identity()) {
  // A simple surface mesh comprises of two triangles with vertices on the
  // coordinate axes and the origin like this:
  //   y
  //   |
  //   |
  //   |
  //   v3(0,15,0)  v2(15,15,0)
  //   +-----------+
  //   |         . |
  //   |  f1  . .  |
  //   |    . .    |
  //   | . .   f0  |
  //   |.          |
  //   +-----------+---------- x
  //   v0(0,0,0)  v1(15,0,0)
  //
  // In the picture above, the positions are expressed in a reference M's
  // frame. The optional parameter X_WM will change the vertex positions to W's
  // frame.
  //
  const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
  std::vector<SurfaceFace> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Vector3<T> vertex_data_M[4] = {
      {0., 0., 0.}, {15., 0., 0.}, {15., 15., 0.}, {0., 15., 0.}};
  std::vector<SurfaceVertex<T>> vertices_W;
  for (int v = 0; v < 4; ++v) vertices_W.emplace_back(X_WM * vertex_data_M[v]);
  auto surface_mesh_W =
      std::make_unique<SurfaceMesh<T>>(move(faces), std::move(vertices_W));

  EXPECT_EQ(2, surface_mesh_W->num_faces());
  EXPECT_EQ(4, surface_mesh_W->num_vertices());
  for (int v = 0; v < 4; ++v)
    EXPECT_EQ(X_WM * vertex_data_M[v],
              surface_mesh_W->vertex(SurfaceVertexIndex(v)).r_MV());
  for (int f = 0; f < 2; ++f)
    for (int v = 0; v < 3; ++v)
      EXPECT_EQ(face_data[f][v],
                surface_mesh_W->element(SurfaceFaceIndex(f)).vertex(v));
  return surface_mesh_W;
}

// Test instantiation of SurfaceMesh using `double` as the underlying scalar
// type.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshDouble) {
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

GTEST_TEST(SurfaceMeshTest, TestSurfaceMeshDouble) {
  auto surface_mesh = TestSurfaceMesh<double>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshAutoDiffXd) {
  auto surface_mesh = GenerateTwoTriangleMesh<AutoDiffXd>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

// Checks the area calculations.
GTEST_TEST(SurfaceMeshTest, TestArea) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  EXPECT_NEAR(surface_mesh->area(SurfaceFaceIndex(0)), 0.5, tol);
  EXPECT_NEAR(surface_mesh->area(SurfaceFaceIndex(1)), 1.0, tol);
  EXPECT_NEAR(surface_mesh->total_area(), 1.5, tol);

  // Verify that the empty mesh and the zero area mesh both give zero area.
  EXPECT_NEAR(GenerateEmptyMesh()->total_area(), 0.0, tol);
  EXPECT_NEAR(GenerateZeroAreaMesh()->total_area(), 0.0, tol);
}

// Checks the centroid calculations.
GTEST_TEST(SurfaceMeshTest, TestCentroid) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  const Vector3<double> centroid = surface_mesh->centroid();
  EXPECT_NEAR(centroid[0], 1.0/6, tol);
  EXPECT_NEAR(centroid[1], -1.0/6, tol);
  EXPECT_NEAR(centroid[2], -0.5, tol);

  // The documentation for the centroid method specifies particular behavior
  // when the total area is zero. Test that.
  EXPECT_NEAR(GenerateEmptyMesh()->centroid().norm(), 0.0, tol);
  EXPECT_NEAR(GenerateZeroAreaMesh()->centroid().norm(), 0.0, tol);
}

GTEST_TEST(SurfaceMeshTest, TestSurfaceMeshAutoDiffXd) {
  auto surface_mesh = TestSurfaceMesh<AutoDiffXd>();
}

template <typename T>
void TestCalcBarycentric() {
  const math::RigidTransform<T> X_WM(
      math::RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto surface_mesh_W = TestSurfaceMesh<T>(X_WM);
  // Empirically the std::numeric_limits<double>::epsilon() is too small to
  // account for the pose.
  const T kTolerance(1e-14);
  const SurfaceFaceIndex f0(0);

  // At v1.
  {
    const Vector3<T> p_M(15., 0., 0.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(0., 1., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Twice closer to v0 than v1.
  {
    const Vector3<T> p_M(5., 0., 0);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(2. / 3., 1. / 3.,
                                                            0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Generic position in the triangle.
  {
    const Vector3<T> p_M(10., 3., 0);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(1. / 3, 7. / 15.,
                                                            1. / 5.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Outside but still on the plane of the triangle.
  {
    const Vector3<T> p_M(30., 7.5, 0.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(-1., 1.5, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Out of the plane of the triangle but still projected into the triangle.
  {
    const Vector3<T> above_midpoint(10., 3., 27.);
    auto barycentric =
        surface_mesh_W->CalcBarycentric(X_WM * above_midpoint, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(1. / 3, 7. / 15.,
                                                            1. / 5.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Out of the plane of the triangle and projected outside the triangle.
  {
    const Vector3<T> p_M(30., 7.5, 27.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(-1., 1.5, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
}

GTEST_TEST(SurfaceMeshTest, TestCalcBarycentricDouble) {
  TestCalcBarycentric<double>();
}

GTEST_TEST(SurfaceMeshTest, TestCalcBarycentricAutoDiffXd) {
  TestCalcBarycentric<AutoDiffXd>();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
