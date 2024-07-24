#include "drake/geometry/proximity/volume_mesh_topology.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {

namespace {

// Test instantiation of VolumeMeshTopology of a geometry M and inspecting its
// components.
template <typename T>
std::unique_ptr<VolumeMeshTopology<T>> TestVolumeMeshTopology() {
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
  // In the picture above, the positions are expressed in M's frame.
  const int element_data[2][4] = {{0, 1, 2, 3}, {0, 2, 1, 4}};
  std::vector<VolumeElement> elements;
  for (int e = 0; e < 2; ++e) elements.emplace_back(element_data[e]);
  const Vector3<T> vertex_data[5] = {Vector3<T>::Zero(), Vector3<T>::UnitX(),
                                     Vector3<T>::UnitY(), Vector3<T>::UnitZ(),
                                     -Vector3<T>::UnitZ()};
  std::vector<Vector3<T>> vertices_W;
  for (int v = 0; v < 5; ++v) {
    vertices_W.emplace_back(vertex_data[v]);
  }
  auto volume_mesh_W = std::make_unique<VolumeMesh<T>>(std::move(elements),
                                                       std::move(vertices_W));

  auto volume_mesh_topology =
      std::make_unique<VolumeMeshTopology<T>>(volume_mesh_W.get());
  // The only shared face is (v0, v1, v2) across from v3 (index 3 face of
  // element 0) and across from v4 (index 3 face of element 1). All other
  // indices are -1 indicating boundary.
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 0), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 1), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 2), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 3), 1);
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 0), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 1), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 2), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 3), 0);

  return volume_mesh_topology;
}

// Test instantiation of VolumeMeshTopology of a slightly more complicated
// geometry M and inspecting its components.
template <typename T>
std::unique_ptr<VolumeMeshTopology<T>> TestVolumeMeshOctahedronTopology() {
  // A volume mesh of an octahedron:
  //      +Z
  //       |
  //       v5
  //       |
  //       |
  //     v0+------v3---+Y
  //      /|
  //     / |
  //   v1  |     v2
  //   /   |
  // +X    v4
  //       |
  //      -Z
  //
  const int element_data[4][4] = {
      {0, 1, 3, 5}, {1, 2, 3, 5}, {0, 3, 1, 4}, {1, 3, 2, 4}};
  std::vector<VolumeElement> elements;
  for (int e = 0; e < 4; ++e) elements.emplace_back(element_data[e]);
  const Vector3<T> vertex_data[6] = {Vector3<T>::Zero(),
                                     Vector3<T>::UnitX(),
                                     Vector3<T>::UnitX() + Vector3<T>::UnitY(),
                                     Vector3<T>::UnitY(),
                                     -Vector3<T>::UnitZ(),
                                     Vector3<T>::UnitZ()};
  std::vector<Vector3<T>> vertices_W;
  for (int v = 0; v < 6; ++v) {
    vertices_W.emplace_back(vertex_data[v]);
  }
  auto volume_mesh_W = std::make_unique<VolumeMesh<T>>(std::move(elements),
                                                       std::move(vertices_W));

  auto volume_mesh_topology =
      std::make_unique<VolumeMeshTopology<T>>(volume_mesh_W.get());
  // There are four shared faces:
  // (v1, v3, v5) is shared by tet 0 at face 0 and tet 1 at face 1
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 0), 1);
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 1), 0);
  // (v1, v2, v3) is shared by tet 1 at face 3 and tet 3 at face 3
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 3), 3);
  EXPECT_EQ(volume_mesh_topology->neighbor(3, 3), 1);
  // (v1, v3, v4) is shared by tet 2 at face 0 and tet 3 at face 2
  EXPECT_EQ(volume_mesh_topology->neighbor(2, 0), 3);
  EXPECT_EQ(volume_mesh_topology->neighbor(3, 2), 2);
  // (v0, v1, v3) is shared by tet 0 at face 3 and tet 2 at face 3
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 3), 2);
  EXPECT_EQ(volume_mesh_topology->neighbor(2, 3), 0);
  // All other neighbors are -1, indicating boundary
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 1), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(0, 2), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 0), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(1, 2), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(2, 1), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(2, 2), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(3, 0), -1);
  EXPECT_EQ(volume_mesh_topology->neighbor(3, 1), -1);

  return volume_mesh_topology;
}

// Test instantiation of VolumeMeshTopology using `double` as the underlying
// scalar type.
GTEST_TEST(VolumeMeshTopologyTest, TestVolumeMeshTopologyDouble) {
  auto volume_mesh_topology = TestVolumeMeshTopology<double>();
  auto volume_mesh_octahedron_topology =
      TestVolumeMeshOctahedronTopology<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(VolumeMeshTopologyTest, TestVolumeMeshTopologyAutoDiffXd) {
  auto volume_mesh_topology = TestVolumeMeshTopology<AutoDiffXd>();
  auto volume_mesh_octahedron_topology =
      TestVolumeMeshOctahedronTopology<AutoDiffXd>();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
