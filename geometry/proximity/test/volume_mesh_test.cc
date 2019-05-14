#include "drake/geometry/proximity/volume_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/mesh_field_linear.h"

namespace drake {
namespace geometry {

// Test instantiation of VolumeMesh and inspecting its components.
template <typename T>
std::unique_ptr<VolumeMesh<T>> TestVolumeMesh();

// Test instantiation of VolumeMesh using `double` as the underlying scalar
// type.
GTEST_TEST(VolumeMeshTest, TestVolumeMeshDouble) {
  auto volume_mesh = TestVolumeMesh<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(VolumeMeshTest, TestVolumeMeshAutoDiffXd) {
  auto volume_mesh = TestVolumeMesh<AutoDiffXd>();
}

template <typename T>
std::unique_ptr<VolumeMesh<T>> TestVolumeMesh() {
  // A  trivial volume mesh comprises of two tetrahedral elements with
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
  const int element_data[2][4] = {{0, 1, 2, 3}, {0, 2, 1, 4}};
  std::vector<VolumeElement> elements;
  for (int e = 0; e < 2; ++e) elements.emplace_back(element_data[e]);
  const Vector3<T> vertex_data[5] = {Vector3<T>::Zero(), Vector3<T>::UnitX(),
                                     Vector3<T>::UnitY(), Vector3<T>::UnitZ(),
                                     -Vector3<T>::UnitZ()};
  std::vector<VolumeVertex<T>> vertices;
  for (int v = 0; v < 5; ++v) vertices.emplace_back(vertex_data[v]);
  auto volume_mesh =
      std::make_unique<VolumeMesh<T>>(std::move(elements), std::move(vertices));

  EXPECT_EQ(2, volume_mesh->num_elements());
  EXPECT_EQ(5, volume_mesh->num_vertices());
  for (int v = 0; v < 5; ++v)
    EXPECT_EQ(vertex_data[v], volume_mesh->vertex(VolumeVertexIndex(v)).r_MV());
  for (int e = 0; e < 2; ++e)
    for (int v = 0; v < 4; ++v)
      EXPECT_EQ(element_data[e][v],
                volume_mesh->element(VolumeElementIndex(e)).vertex(v));
  return volume_mesh;
}


// Tests instantiation of VolumeMeshField and evaluating a scalar field value.
template <typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TestVolumeMeshField();

// Test instantiation of VolumeMeshField using `double` as the underlying
// scalar type.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldDouble) {
  auto volume_mesh_field = TestVolumeMeshField<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There are no tests of
// differentiation.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldAutoDiffXd) {
  auto volume_mesh_field = TestVolumeMeshField<AutoDiffXd>();
}

template <typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TestVolumeMeshField() {
  auto volume_mesh = TestVolumeMesh<T>();

  // We give names to the values at vertices for testing later.
  const T p0{1.};
  const T p1{2.};
  const T p2{3.};
  const T p3{4.};
  const T p4{5.};
  std::vector<T> p_values = {p0, p1, p2, p3, p4};

  auto volume_mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "pressure", std::move(p_values), volume_mesh.get());

  // Tests evaluation of the field on the element e0 {v0, v1, v2, v3}.
  const VolumeElementIndex e0(0);
  const typename VolumeMesh<T>::Barycentric b{0.4, 0.3, 0.2, 0.1};
  const T expect_p = b(0) * p0 + b(1) * p1 + b(2) * p2 + b(3) * p3;
  EXPECT_EQ(expect_p, volume_mesh_field->Evaluate(e0, b));

  return volume_mesh_field;
}

}  // namespace geometry
}  // namespace drake
