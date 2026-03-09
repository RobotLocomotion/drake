#include "drake/geometry/proximity/volume_mesh_topology.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {

namespace {

// Test instantiation of VolumeMeshTopology of a geometry M and inspecting its
// components.
GTEST_TEST(VolumeMeshTopologyTest, TestVolumeMeshTopology) {
  // A trivial volume mesh comprises of two tetrahedral elements with
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
  const Vector3<double> vertex_data[5] = {
      Vector3<double>::Zero(), Vector3<double>::UnitX(),
      Vector3<double>::UnitY(), Vector3<double>::UnitZ(),
      -Vector3<double>::UnitZ()};
  std::vector<Vector3<double>> vertices_W;
  for (int v = 0; v < 5; ++v) {
    vertices_W.emplace_back(vertex_data[v]);
  }
  const VolumeMesh<double> volume_mesh_W(std::move(elements),
                                         std::move(vertices_W));

  const VolumeMeshTopology volume_mesh_topology(volume_mesh_W);
  // The only shared face is (v0, v1, v2) across from v3 (index 3 face of
  // element 0) and across from v4 (index 3 face of element 1). All other
  // indices are -1 indicating boundary.
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 0), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 1), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 2), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 3), 1);
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 0), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 1), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 2), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 3), 0);
}

// Test instantiation of VolumeMeshTopology of a slightly more complicated
// geometry M and inspecting its components.
GTEST_TEST(VolumeMeshTopologyTest, TestVolumeMeshOctahedronTopology) {
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
  const Vector3<double> vertex_data[6] = {
      Vector3<double>::Zero(),
      Vector3<double>::UnitX(),
      Vector3<double>::UnitX() + Vector3<double>::UnitY(),
      Vector3<double>::UnitY(),
      -Vector3<double>::UnitZ(),
      Vector3<double>::UnitZ()};
  std::vector<Vector3<double>> vertices_W;
  for (int v = 0; v < 6; ++v) {
    vertices_W.emplace_back(vertex_data[v]);
  }
  const VolumeMesh<double> volume_mesh_W(std::move(elements),
                                         std::move(vertices_W));

  const VolumeMeshTopology volume_mesh_topology(volume_mesh_W);
  // There are four shared faces:
  // (v1, v3, v5) is shared by tet 0 at face 0 and tet 1 at face 1
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 0), 1);
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 1), 0);
  // (v1, v2, v3) is shared by tet 1 at face 3 and tet 3 at face 3
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 3), 3);
  EXPECT_EQ(volume_mesh_topology.neighbor(3, 3), 1);
  // (v1, v3, v4) is shared by tet 2 at face 0 and tet 3 at face 2
  EXPECT_EQ(volume_mesh_topology.neighbor(2, 0), 3);
  EXPECT_EQ(volume_mesh_topology.neighbor(3, 2), 2);
  // (v0, v1, v3) is shared by tet 0 at face 3 and tet 2 at face 3
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 3), 2);
  EXPECT_EQ(volume_mesh_topology.neighbor(2, 3), 0);
  // All other neighbors are -1, indicating boundary
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 1), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(0, 2), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 0), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(1, 2), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(2, 1), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(2, 2), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(3, 0), -1);
  EXPECT_EQ(volume_mesh_topology.neighbor(3, 1), -1);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
