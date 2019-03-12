#include "drake/geometry/query_results/contact_surface.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/mesh_field.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {


// TODO(DamrongGuoy): Consider splitting the test into several smaller tests
//  including a separated mesh test.
// Tests instantiation of ContactSurface and inspecting its components (the
// mesh and the mesh fields). We cannot access its mesh fields directly, so
// we check them by evaluating the field values at certain positions.
template <typename T> ContactSurface<T> TestContactSurface();

// Tests instantiation of ContactSurface using `double` as the underlying
// scalar type.
GTEST_TEST(ContactSurfaceTest, TestContactSurfaceDouble) {
  auto contact_surface = TestContactSurface<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no tests of
// differentiation.
GTEST_TEST(ContactSurfaceTest, TestContactSurfaceAutoDiffXd) {
  auto contact_surface = TestContactSurface<AutoDiffXd>();
}

template <typename T>
ContactSurface<T> TestContactSurface() {
  auto id_M = GeometryId::get_new_id();
  auto id_N = GeometryId::get_new_id();

  // Simple handmade data for a contact surface. It consists of two right
  // triangles that make a square.
  //
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
      std::make_unique<SurfaceMesh<T>>(std::move(faces), std::move(vertices));
  // We record the reference for testing later.
  auto& surface_mesh_ref = *(surface_mesh.get());

  // Increasing values of `e` from one vertex to the next.
  // We give names to the values at vertices for testing later.
  const T e0{0.};
  const T e1{1.};
  const T e2{2.};
  const T e3{3.};
  std::vector<T> e_values = {e0, e1, e2, e3};
  auto e_field = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", std::move(e_values), surface_mesh.get());

  // Slightly different values of grad_h_MN_M at each vertex.
  // We give names to the values at vertices for testing later.
  const Vector3<T> g0(-0.1, -0.1, 1.);
  const Vector3<T> g1(0.1, -0.1, 1.);
  const Vector3<T> g2(0.1, 0.1, 1.);
  const Vector3<T> g3(-0.1, 0.1, 1.);
  std::vector<Vector3<T>> grad_h_MN_M_values = {g0, g1, g2, g3};
  auto grad_h_MN_M_field =
      std::make_unique<SurfaceMeshFieldLinear<Vector3<T>, T>>(
          "grad_h_MN_M", std::move(grad_h_MN_M_values), surface_mesh.get());

  ContactSurface<T> contact_surface(id_M, id_N, std::move(surface_mesh),
                                    std::move(e_field),
                                    std::move(grad_h_MN_M_field));

  // Start testing the ContactSurface<> data structure.
  EXPECT_EQ(id_M, contact_surface.id_M());
  EXPECT_EQ(id_N, contact_surface.id_N());
  // Check memory address of the mesh. We don't want to compare the mesh
  // objects themselves.
  EXPECT_EQ(&surface_mesh_ref, &contact_surface.mesh());
  EXPECT_EQ(2, contact_surface.mesh().num_faces());
  EXPECT_EQ(4, contact_surface.mesh().num_vertices());
  // Tests evaluation of `e` on face f0 {0, 1, 2}.
  {
    const SurfaceFaceIndex f0(0);
    const typename SurfaceMesh<T>::Barycentric b{0.2, 0.3, 0.5};
    const T expect_e = b(0) * e0 + b(1) * e1 + b(2) * e2;
    EXPECT_EQ(expect_e, contact_surface.EvaluateE_MN(f0, b));
  }
  // Tests evaluation of `grad_h_MN_M` on face f1 {2, 3, 0}.
  {
    const SurfaceFaceIndex f1(1);
    const typename SurfaceMesh<T>::Barycentric b{0.6, 0.3, 0.1};
    // On face f1, we have these quantities.
    //---+--------+----------+-----------------
    // v | vertex | grad_h_MN_M | barycentric
    //---+--------+-------------+--------------
    // 0 |   v2   |      g2     |     0.6
    // 1 |   v3   |      g3     |     0.3
    // 2 |   v0   |      g0     |     0.1
    //---+--------+-------------+--------------
    const Vector3<T> expect_g = T(0.6) * g2 + T(0.3) * g3 + T(0.1) * g0;
    EXPECT_EQ(expect_g, contact_surface.EvaluateGrad_h_MN_M(f1, b));
  }
  // Tests area() of triangular faces.
  {
    EXPECT_EQ(T(0.5), contact_surface.mesh().area(SurfaceFaceIndex(0)));
    EXPECT_EQ(T(0.5), contact_surface.mesh().area(SurfaceFaceIndex(1)));
  }

  return contact_surface;
}

}  // namespace geometry
}  // namespace drake

