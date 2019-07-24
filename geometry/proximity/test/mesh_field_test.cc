#include "drake/geometry/proximity/mesh_field.h"

#include <memory>

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

// Tests CloneAndSetMesh(). We use `double` and SurfaceMesh<double>
// to represent the type parameters FieldValue and MeshType respectively.
GTEST_TEST(MeshFieldTest, TestClone) {
  using FieldValue = double;
  using MeshType = SurfaceMesh<double>;
  using MeshFieldBase = MeshField<FieldValue, MeshType>;

  class MeshFieldSubclass : public MeshFieldBase {
   public:
    explicit MeshFieldSubclass(MeshType* mesh): MeshFieldBase(mesh) {}

    FieldValue EvaluateAtVertex(MeshType::VertexIndex) const final {
      return FieldValue(0);
    }

    FieldValue Evaluate(MeshType::ElementIndex,
                        const MeshType::Barycentric&) const final {
      return FieldValue(0);
    }
    FieldValue EvaluateCartesian(
                         MeshType::ElementIndex,
                         const MeshType::Cartesian&) const final {
      return FieldValue(0);
    }
   private:
    DRAKE_NODISCARD std::unique_ptr<MeshFieldBase> DoCloneWithNullMesh() const
    final {
      return std::make_unique<MeshFieldSubclass>(*this);
    }
  };
  auto mesh1 = GenerateMesh<double>();
  MeshFieldSubclass original(mesh1.get());
  MeshType mesh2 = *mesh1;

  std::unique_ptr<MeshFieldBase> clone = original.CloneAndSetMesh(&mesh2);
  EXPECT_EQ(&mesh2, &clone->mesh());
  EXPECT_NE(nullptr, dynamic_cast<MeshFieldSubclass*>(clone.get()));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
