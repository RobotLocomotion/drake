#include "drake/geometry/proximity/surface_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

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
GTEST_TEST(SurfaceMeshTest, TestSurfaceMeshDouble) {
  auto surface_mesh = TestSurfaceMesh<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// diffrentiation.
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



}  // namespace geometry
}  // namespace drake
