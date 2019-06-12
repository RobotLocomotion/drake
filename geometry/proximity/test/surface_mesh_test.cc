#include "drake/geometry/proximity/surface_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

// Test instantiation of SurfaceMesh and inspecting its components.
template <typename T>
std::unique_ptr<SurfaceMesh<T>> TestSurfaceMesh();

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
std::unique_ptr<SurfaceMesh<T>> TestSurfaceMesh() {
  // A simple surface mesh comprises of two triangles with vertices on the
  // coordinate axes and the origin like this:
  //   y
  //   |
  //   |
  //   |
  //   v3(0,1,0)  v2(1,1,0)
  //   +-----------+
  //   |         . |
  //   |  f1  . .  |
  //   |    . .    |
  //   | . .   f0  |
  //   |.          |
  //   +-----------+---------- x
  //   v0(0,0,0)  v1(1,0,0)
  //
  const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
  std::vector<SurfaceFace> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Vector3<T> vertex_data[4] = {
      {0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
  std::vector<SurfaceVertex<T>> vertices;
  for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
  auto surface_mesh =
      std::make_unique<SurfaceMesh<T>>(move(faces), std::move(vertices));

  EXPECT_EQ(2, surface_mesh->num_faces());
  EXPECT_EQ(4, surface_mesh->num_vertices());
  for (int v = 0; v < 4; ++v)
    EXPECT_EQ(vertex_data[v],
              surface_mesh->vertex(SurfaceVertexIndex(v)).r_MV());
  for (int f = 0; f < 2; ++f)
    for (int v = 0; v < 3; ++v)
      EXPECT_EQ(face_data[f][v],
                surface_mesh->element(SurfaceFaceIndex(f)).vertex(v));
  return surface_mesh;
}

template <typename T>
void TestCalcBarycentric();

GTEST_TEST(SurfaceMeshTest, TestCalcBarycentricDouble) {
  TestCalcBarycentric<double>();
}

template <typename T>
void TestCalcBarycentric() {
  auto surface_mesh = TestSurfaceMesh<T>();
  const T kTolerance(std::numeric_limits<double>::epsilon());
  const SurfaceFaceIndex f0(0);

  // Midpoint between v0 and v2.
  {
    const Vector3<T> midpoint(0.5, 0.5, 0);
    auto barycentric = surface_mesh->CalcBarycentric(midpoint, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(0.5, 0, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Point above the midpoint between v0 and v2.
  {
    const Vector3<T> above_midpoint(0.5, 0.5, 10.);
    auto barycentric = surface_mesh->CalcBarycentric(above_midpoint, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(0.5, 0, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
}

}  // namespace geometry
}  // namespace drake
