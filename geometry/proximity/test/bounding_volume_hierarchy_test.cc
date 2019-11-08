#include "drake/geometry/proximity/bounding_volume_hierarchy.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
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
  static Vector3<double> ComputeCentroid(const MeshType* mesh,
      const typename MeshType::ElementIndex i) {
    return BoundingVolumeHierarchy<MeshType>::ComputeCentroid(mesh, i);
  }

  template <class MeshType>
  static AABB ComputeBoundingVolume(
      const MeshType* mesh,
      const typename std::vector<
          typename BoundingVolumeHierarchy<MeshType>::CentroidPair>::iterator
          start,
      const typename std::vector<
          typename BoundingVolumeHierarchy<MeshType>::CentroidPair>::iterator
          end) {
    return BoundingVolumeHierarchy<MeshType>::ComputeBoundingVolume(mesh, start,
                                                                    end);
  }

  template <class MeshType>
  static BVNode<MeshType>* GetBVTree(
      const BoundingVolumeHierarchy<MeshType>& bvh) {
    return bvh.root_node_.get();
  }
};

namespace {

// Test fixture class for testing BoundingVolumeHierarchy. Uses a coarse
// sphere, i.e. an octahedron, as the underlying mesh.
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

// Tests computing the bounding volume of elements.
TEST_F(BVHTest, TestComputeBoundingVolume) {
  std::vector<std::pair<SurfaceFaceIndex, Vector3<double>>> element_centroids;

  // Add all the elements from the octahedron to test multiple elements. The
  // centroid values are irrelevant for this test, but the bounding box should
  // encompass the whole sphere with a center of 0 and half width of 1.5.
  for (SurfaceFaceIndex i(0); i < mesh_.num_elements(); ++i) {
    element_centroids.emplace_back(i, Vector3<double>(0.5, 0.5, 0.5));
  }
  AABB aabb = BVHTester::ComputeBoundingVolume<SurfaceMesh<double>>(
      &mesh_, element_centroids.begin(), element_centroids.end());
  EXPECT_TRUE(CompareMatrices(aabb.center(), Vector3<double>(0., 0., 0.)));
  EXPECT_TRUE(
      CompareMatrices(aabb.half_width(), Vector3<double>(1.5, 1.5, 1.5)));
}

// Tests properties from building the bounding volume tree.
TEST_F(BVHTest, TestBuildBVTree) {
  // Since it's a binary tree with a single element at each leaf node, the tree
  // depth can be found using 2^d = num_elements. The octahedron has 8 elements
  // so we should end up with a balanced tree of depth 3 where each level's
  // volume is less than its parent's volume.
  BVNode<SurfaceMesh<double>>* bv_tree = BVHTester::GetBVTree(bvh_);
  int num_elements = mesh_.num_elements();
  std::set<SurfaceFaceIndex> element_indices;
  std::function<void(BVNode<SurfaceMesh<double>>*, int)> check_node;
  check_node = [&check_node, &element_indices, num_elements](
                   BVNode<SurfaceMesh<double>>* node, int depth) {
    if (depth < 3) {
      double node_volume = node->aabb.CalcVolume();
      Vector3<double> node_bounds_upper =
          node->aabb.center() + node->aabb.half_width();
      Vector3<double> node_bounds_lower =
          node->aabb.center() - node->aabb.half_width();

      auto check_child = [&check_node, &node_volume, &node_bounds_upper,
                          &node_bounds_lower](
                             BVNode<SurfaceMesh<double>>* child, int depth) {
        EXPECT_NE(child, nullptr);
        // Check the volume is less than the parent.
        double child_volume = child->aabb.CalcVolume();
        EXPECT_LE(child_volume, node_volume);
        // Check the bounds are within the parent.
        Vector3<double> child_bounds_upper =
            child->aabb.center() + child->aabb.half_width();
        Vector3<double> child_bounds_lower =
            child->aabb.center() - child->aabb.half_width();
        EXPECT_TRUE(
            CompareMatrices(node_bounds_upper.cwiseMin(child_bounds_upper),
                            child_bounds_upper));
        EXPECT_TRUE(
            CompareMatrices(node_bounds_lower.cwiseMax(child_bounds_lower),
                            child_bounds_lower));
        // Check each of the branches at increasing depth.
        check_node(child, depth + 1);
      };
      check_child(node->left.get(), depth);
      check_child(node->right.get(), depth);
    } else {
      // At depth 3, we should reach the leaf node with a valid and unique
      // element instead of more branches.
      EXPECT_EQ(node->left, nullptr);
      EXPECT_EQ(node->right, nullptr);
      EXPECT_GE(node->element_index, 0);
      EXPECT_LT(node->element_index, num_elements);
      EXPECT_EQ(element_indices.count(node->element_index), 0);
      element_indices.insert(node->element_index);
    }
  };
  check_node(bv_tree, 0);
  // Check that we found a leaf node for all elements.
  EXPECT_EQ(element_indices.size(), num_elements);
}

// Tests copy constructor.
TEST_F(BVHTest, TestCopy) {
  // Copy constructor.
  BoundingVolumeHierarchy<SurfaceMesh<double>> copy(bvh_);

  // Confirm that it's a deep copy.
  EXPECT_NE(BVHTester::GetBVTree(copy), nullptr);
  std::function<void(BVNode<SurfaceMesh<double>>*,
                     BVNode<SurfaceMesh<double>>*)> check_copy;
  check_copy = [&check_copy](BVNode<SurfaceMesh<double>>* orig,
                             BVNode<SurfaceMesh<double>>* copy) {
    EXPECT_NE(orig, copy);
    if (orig->left != nullptr) {
      check_copy(orig->left.get(), copy->left.get());
    }
    if (orig->right != nullptr) {
      check_copy(orig->right.get(), copy->right.get());
    }
  };
  check_copy(BVHTester::GetBVTree(bvh_), BVHTester::GetBVTree(copy));
}

// Tests computing the centroid of an element.
GTEST_TEST(BoundingVolumeHierarchyTest, TestComputeCentroid) {
  // Set resolution at double so that we get the coarsest mesh of 8 elements.
  auto mesh = MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6);
  Vector3<double> centroid = BVHTester::ComputeCentroid<SurfaceMesh<double>>(
      &mesh, SurfaceFaceIndex(0));
  // The first face of our octahedron with is a triangle with vertices at 1, 2,
  // and 3 along each respective axis, so its centroid should average out to
  // 1/3, 2/3, and 3/3.
  EXPECT_TRUE(CompareMatrices(centroid, Vector3<double>(1./3., 2./3., 1.)));
}

// Tests calculating the bounding box volume.
GTEST_TEST(AABBTest, TestVolume) {
  AABB aabb = AABB(Vector3<double>(-1, 2, 1), Vector3<double>(2, 0.5, 2.7));
  EXPECT_EQ(aabb.CalcVolume(), 21.6);
  AABB zero_aabb = AABB(Vector3<double>(3, -4, 1.3), Vector3<double>(0, 0, 0));
  EXPECT_EQ(zero_aabb.CalcVolume(), 0);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
