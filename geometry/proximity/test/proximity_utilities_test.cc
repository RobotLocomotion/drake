#include "drake/geometry/proximity/proximity_utilities.h"

#include <memory>
#include <sstream>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using fcl::CollisionGeometryd;
using fcl::CollisionObjectd;

// Construct a couple of encodings and test their properties.
GTEST_TEST(EncodedData, ConstructorAndProperties) {
  GeometryId id_A = GeometryId::get_new_id();
  EncodedData data_A(id_A, true);
  EXPECT_EQ(data_A.id(), id_A);
  EXPECT_TRUE(data_A.is_dynamic());

  GeometryId id_B = GeometryId::get_new_id();
  EncodedData data_B(id_B, false);
  EXPECT_EQ(data_B.id(), id_B);
  EXPECT_FALSE(data_B.is_dynamic());
}

GTEST_TEST(EncodedData, FactoryConstruction) {
  GeometryId id_A = GeometryId::get_new_id();

  EncodedData dynamic = EncodedData::encode_dynamic(id_A);
  EXPECT_EQ(dynamic.id(), id_A);
  EXPECT_TRUE(dynamic.is_dynamic());

  EncodedData anchored = EncodedData::encode_anchored(id_A);
  EXPECT_EQ(anchored.id(), id_A);
  EXPECT_FALSE(anchored.is_dynamic());
}

// This tests writing to and extracting from an fcl object,
GTEST_TEST(EncodedData, ConstructionFromFclObject) {
  GeometryId id_A = GeometryId::get_new_id();
  EncodedData data_A(id_A, true);
  CollisionObjectd object(std::shared_ptr<CollisionGeometryd>(nullptr));

  data_A.write_to(&object);
  {
    EncodedData data_read(object);
    EXPECT_TRUE(data_read.is_dynamic());
    EXPECT_EQ(data_read.id(), data_A.id());
  }

  data_A.set_anchored();
  data_A.write_to(&object);
  {
    EncodedData data_read(object);
    EXPECT_FALSE(data_read.is_dynamic());
    EXPECT_EQ(data_read.id(), data_A.id());
  }
}

class VolumeMeshUtilities : public ::testing::Test {
 public:
  void SetUp() override {
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
    const Vector3<double> vertex_data[5] = {
        Vector3<double>::Zero(), Vector3<double>::UnitX(),
        Vector3<double>::UnitY(), Vector3<double>::UnitZ(),
        -Vector3<double>::UnitZ()};
    std::vector<Vector3d> vertices;
    for (int v = 0; v < 5; ++v) vertices.emplace_back(vertex_data[v]);
    volume_mesh = std::make_unique<VolumeMesh<double>>(std::move(elements),
                                                       std::move(vertices));
  }

 protected:
  std::unique_ptr<VolumeMesh<double>> volume_mesh;
};

TEST_F(VolumeMeshUtilities, CountEdges) {
  const int expected_edges = 9;
  const int edges = CountEdges(*volume_mesh);
  EXPECT_EQ(expected_edges, edges);
}

TEST_F(VolumeMeshUtilities, CountFaces) {
  const int expected_faces = 7;
  const int faces = CountFaces(*volume_mesh);
  EXPECT_EQ(expected_faces, faces);
}

TEST_F(VolumeMeshUtilities, ComputeEulerCharacteristic) {
  // For a convex mesh that is homeomorphic to a 3-dimensional ball, χ = 1.
  const int expected_characteristic = 1;
  const int characteristic = ComputeEulerCharacteristic(*volume_mesh);
  EXPECT_EQ(expected_characteristic, characteristic);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
