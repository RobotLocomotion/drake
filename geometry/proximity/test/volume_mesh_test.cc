#include "drake/geometry/proximity/volume_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

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

}  // namespace geometry
}  // namespace drake
