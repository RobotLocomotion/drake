#include "drake/geometry/query_results/contact_surface.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/mesh_field.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {

// Tests instantiation of ContactSurface and evaluating its field variables.
GTEST_TEST(ContactSurfaceTest, TestInstantiation) {
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
  const Vector3<double> vertex_data[4] = {
      {0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
  std::vector<SurfaceVertex<double>> vertices;
  for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
  auto surface_mesh = std::make_shared<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
  // Increasing values of `e` from one vertex to the next.
  // We give names to the values at vertices for testing later.
  const double e0 = 0.;
  const double e1 = 1.;
  const double e2 = 2.;
  const double e3 = 3.;
  std::vector<double> e_values = {e0, e1, e2, e3};
  SurfaceMeshField<double, double> e_field("e", std::move(e_values),
                                           surface_mesh);
  // Slightly different values of grad_h_M at each vertex.
  // We give names to the values at vertices for testing later.
  const Vector3<double> g0(-0.1, -0.1, 1.);
  const Vector3<double> g1(0.1, -0.1, 1.);
  const Vector3<double> g2(0.1, 0.1, 1.);
  const Vector3<double> g3(-0.1, 0.1, 1.);
  std::vector<Vector3<double>> grad_h_M_values = {g0, g1, g2, g3};
  SurfaceMeshField<Vector3<double>, double> grad_h_M_field(
      "grad_h_M", std::move(grad_h_M_values), surface_mesh);
  ContactSurface<double> contact_surface(
      id_M, id_N, surface_mesh, std::move(e_field), std::move(grad_h_M_field));
  EXPECT_EQ(id_M, contact_surface.id_M());
  EXPECT_EQ(id_N, contact_surface.id_N());
  EXPECT_EQ(2, contact_surface.num_faces());
  EXPECT_EQ(4, contact_surface.num_vertices());
  // We will test evaluation of `e` on face f0 {0, 1, 2}.
  {
    const SurfaceFaceIndex f0(0);
    const Vector2<double> s{0.3, 0.5};
    // On f0, 1-s(0)-s(1) corresponds to vertex v0, s(0) corresponds to
    // vertex v1, and s(1) corresponds to vertex v2.
    const double b0 = 1.0 - s(0) - s(1);
    const double b1 = s(0);
    const double b2 = s(1);
    const double expect_e = b0 * e0 + b1 * e1 + b2 * e2;
    EXPECT_EQ(expect_e, contact_surface.e(f0, s));
  }
  // We will test evaluation of `grad_h_M` on face f1 {2, 3, 0}.
  {
    const SurfaceFaceIndex f1(1);
    const Vector2<double> s{0.3, 0.1};
    // On f1, 1-s(0)-s(1) corresponds to vertex v2, s(0) corresponds to
    // vertex v3, and s(1) corresponds to vertex v0.
    const double b2 = 1 - s(0) - s(1);
    const double b3 = s(0);
    const double b0 = s(1);
    const Vector3<double> expect_g = b2 * g2 + b3 * g3 + b0 * g0;
    EXPECT_EQ(expect_g, contact_surface.grad_h_M(f1, s));
  }
}

}  // namespace geometry
}  // namespace drake

