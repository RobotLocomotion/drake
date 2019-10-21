#include "drake/geometry/proximity/mesh_field_linear.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

template <typename T>
std::unique_ptr<SurfaceMesh<T>> GenerateMesh() {
// A simple surface mesh consists of two right triangles that make a square.
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
      std::make_unique<SurfaceMesh<T>>(move(faces), std::move(vertices));
  return surface_mesh;
}

// Tests Evaluate(VertexIndex).
GTEST_TEST(MeshFieldLinearTest, EvaluateAtVertex) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e", std::move(e_values), mesh.get());
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(0)), 0);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(1)), 1);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(2)), 2);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(3)), 3);
}

// Tests CloneAndSetMesh(). We use `double` and SurfaceMesh<double> as
// representative arguments for type parameters.
GTEST_TEST(MeshFieldLinearTest, TestDoCloneWithMesh) {
  using FieldValue = double;
  using MeshType = SurfaceMesh<double>;
  using MeshFieldLineard = MeshFieldLinear<FieldValue, MeshType>;

  auto mesh1 = GenerateMesh<double>();
  std::vector<FieldValue> e_values = {0., 1., 2., 3.};
  auto original =
      std::make_unique<MeshFieldLineard>("e", std::move(e_values), mesh1.get());

  MeshType mesh2 = *mesh1;
  auto clone_base = original->CloneAndSetMesh(&mesh2);
  auto clone = dynamic_cast<MeshFieldLineard*>(clone_base.get());

  // Check uniqueness.
  EXPECT_NE(original.get(), clone);

  // Check equivalence.
  EXPECT_EQ(original->name(), clone->name());
  EXPECT_EQ(original->values(), clone->values());
}

// Tests Equal.
GTEST_TEST(MeshFieldLinearTest, TestEqual) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e", std::move(e_values), mesh.get());

  // Same field.
  auto field0 = mesh_field->CloneAndSetMesh(mesh.get());
  EXPECT_TRUE(mesh_field->Equal(*field0));

  // Different mesh.
  SurfaceMesh<double> alt_mesh = *mesh;
  alt_mesh.ReverseFaceWinding();
  auto field1 = mesh_field->CloneAndSetMesh(&alt_mesh);
  EXPECT_FALSE(mesh_field->Equal(*field1));

  // Different e values.
  std::vector<double> alt_e_values = {3., 2., 1., 0.};
  auto field2 = MeshFieldLinear<double, SurfaceMesh<double>>(
      "e", std::move(alt_e_values), mesh.get());
  EXPECT_FALSE(mesh_field->Equal(field2));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
