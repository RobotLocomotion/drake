#include "drake/geometry/proximity/bounding_volume_hierarchy.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// Friend class for accessing BoundingVolumeHierarchy's protected/private
// functionality.
class BVHTester {
 public:
  BVHTester() = delete;

  template <class MeshType>
  static Vector3<double> ComputeCentroid(
      const BoundingVolumeHierarchy<MeshType>& bvh,
      const typename MeshType::ElementIndex i) {
    return bvh.ComputeCentroid(i);
  }

  template <class MeshType>
  static const AABB ComputeBoundingVolume(
      const BoundingVolumeHierarchy<MeshType>& bvh,
      const typename std::vector<
          typename BoundingVolumeHierarchy<MeshType>::CentroidPair>::iterator
          start,
      const typename std::vector<
          typename BoundingVolumeHierarchy<MeshType>::CentroidPair>::iterator
          end) {
    return bvh.ComputeBoundingVolume(start, end);
  }

  template <class MeshType>
  static BVNode<MeshType>* GetBVTree(
      const BoundingVolumeHierarchy<MeshType>& bvh) {
    return bvh.bv_tree_.get();
  }
};

namespace {

// Helper class for testing BoundingVolumeHierarchy. Uses a coarse sphere, i.e.
// an octahedron, as the underlying mesh.
class BVHTest : public ::testing::Test {
 public:
  BVHTest()
      : ::testing::Test(),
        mesh_(MakeSphereSurfaceMesh<double>(Sphere(1.5), 3)),
        bvh_(BoundingVolumeHierarchy<SurfaceMesh<double>>(&mesh_)) {}

 protected:
  SurfaceMesh<double> mesh_;
  BoundingVolumeHierarchy<SurfaceMesh<double>> bvh_;
};

// Tests computing the centroid of an element.
TEST_F(BVHTest, TestComputeCentroid) {
  Vector3<double> centroid = BVHTester::ComputeCentroid<SurfaceMesh<double>>(
      bvh_, SurfaceFaceIndex(0));
  // The first face of our octahedron with radius 1.5 is a triangle with
  // vertices at 1.5 along each axis, so its centroid should average out to
  // 1.5 / 3 = 0.5.
  EXPECT_EQ(centroid, Vector3<double>(0.5, 0.5, 0.5));
}

// Tests computing the bounding volume of an element.
TEST_F(BVHTest, TestComputeBoundingVolume) {
  std::vector<BoundingVolumeHierarchy<SurfaceMesh<double>>::CentroidPair>
          element_centroids;
  element_centroids.push_back(
      std::pair(SurfaceFaceIndex(0), Vector3<double>(0.5, 0.5, 0.5)));
  const AABB aabb = BVHTester::ComputeBoundingVolume<SurfaceMesh<double>>(
      bvh_, element_centroids.begin(), element_centroids.end());
  // The first face of our octahedron with radius 1.5 is a triangle with
  // vertices at 1.5 along each axis, so the bounding box should cover a volume
  // of 0 to 1.5. Thus it should have a center of 1.5 / 2 = 0.75 and halfwidths
  // of 0.75.
  EXPECT_EQ(aabb.center(), Vector3<double>(0.75, 0.75, 0.75));
  EXPECT_EQ(aabb.halfwidth(), Vector3<double>(0.75, 0.75, 0.75));
}

// Tests properties from building the bounding volume tree.
TEST_F(BVHTest, TestBuildBVTree) {
  // Since its a binary tree with a single element at each leaf node, the tree
  // depth can be found using 2^d = num_elements. The octahedron has 8 elements
  // so we should end up with a balanced tree of depth 3 where each level's
  // volume is less than its parent's volume.
  BVNode<SurfaceMesh<double>>* bv_tree = BVHTester::GetBVTree(bvh_);
  int num_elements = mesh_.num_elements();
  std::function<void(BVNode<SurfaceMesh<double>>*, int, int)> check_node;
  check_node = [&check_node, num_elements](BVNode<SurfaceMesh<double>>* node,
                                           int depth, int volume) -> void {
    if (depth < 3) {
      EXPECT_NE(node->left, nullptr);
      EXPECT_NE(node->right, nullptr);

      // Check each of the branches at increasing depth.
      int left_volume = node->left->aabb.volume();
      EXPECT_LE(left_volume, volume);
      check_node(node->left.get(), depth + 1, left_volume);
      int right_volume = node->right->aabb.volume();
      EXPECT_LE(right_volume, volume);
      check_node(node->right.get(), depth + 1, right_volume);
    } else {
      // At depth 3, we should reach the leaf node with a valid element instead
      // of more branches.
      EXPECT_EQ(node->left, nullptr);
      EXPECT_EQ(node->right, nullptr);
      EXPECT_GE(node->e, 0);
      EXPECT_LT(node->e, num_elements);
    }
  };
  check_node(bv_tree, 0, bv_tree->aabb.volume());
}



// Tests calculating the bounding box volume.
GTEST_TEST(AABBTest, TestVolume) {
  AABB aabb = AABB(Vector3<double>(-1, 2, 1), Vector3<double>(2, 0.5, 2.7));
  EXPECT_EQ(aabb.volume(), 21.6);
  AABB zero_aabb = AABB(Vector3<double>(0, 0, 0), Vector3<double>(0, 0, 0));
  EXPECT_EQ(zero_aabb.volume(), 0);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
