#include "drake/geometry/proximity/bounding_volume_hierarchy.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;

// Friend class for accessing BoundingVolumeHierarchy's protected/private
// functionality.
class BVHTester {
 public:
  BVHTester() = delete;

  template <class MeshType>
  static Vector3d ComputeCentroid(const MeshType& mesh,
                                  const typename MeshType::ElementIndex i) {
    return BoundingVolumeHierarchy<MeshType>::ComputeCentroid(mesh, i);
  }

  template <class MeshType>
  static Aabb ComputeBoundingVolume(
      const MeshType& mesh,
      const typename std::vector<
          typename BoundingVolumeHierarchy<MeshType>::CentroidPair>::iterator
          start,
      const typename std::vector<
          typename BoundingVolumeHierarchy<MeshType>::CentroidPair>::iterator
          end) {
    return BoundingVolumeHierarchy<MeshType>::ComputeBoundingVolume(mesh, start,
                                                                    end);
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
        bvh_(BoundingVolumeHierarchy<SurfaceMesh<double>>(mesh_)) {}

 protected:
  SurfaceMesh<double> mesh_;
  BoundingVolumeHierarchy<SurfaceMesh<double>> bvh_;
};

// Tests computing the bounding volume of elements.
TEST_F(BVHTest, TestComputeBoundingVolume) {
  std::vector<std::pair<SurfaceFaceIndex, Vector3d>> tri_centroids;

  // Add all the elements from the octahedron to test multiple elements. The
  // centroid values are irrelevant for this test, but the bounding box should
  // encompass the whole sphere with a center of 0 and half width of 1.5.
  for (SurfaceFaceIndex i(0); i < mesh_.num_elements(); ++i) {
    tri_centroids.emplace_back(i, Vector3d(0.5, 0.5, 0.5));
  }
  Aabb aabb = BVHTester::ComputeBoundingVolume<SurfaceMesh<double>>(
      mesh_, tri_centroids.begin(), tri_centroids.end());
  EXPECT_TRUE(CompareMatrices(aabb.center(), Vector3d(0., 0., 0.)));
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), Vector3d(1.5, 1.5, 1.5)));

  // Test with a volume mesh. As above, the centroids are still irrelevant. The
  // bounding box should encompass the whole ellipsoid with a center of 0 and
  // half width of 1, 2, 3.
  std::vector<std::pair<VolumeElementIndex, Vector3d>> tet_centroids;
  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(Ellipsoid(1., 2., 3.), 6);
  for (VolumeElementIndex i(0); i < volume_mesh.num_elements(); ++i) {
    tet_centroids.emplace_back(i, Vector3d(0.5, 0.5, 0.5));
  }
  aabb = BVHTester::ComputeBoundingVolume<VolumeMesh<double>>(
      volume_mesh, tet_centroids.begin(), tet_centroids.end());
  EXPECT_TRUE(CompareMatrices(aabb.center(), Vector3d(0., 0., 0.)));
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), Vector3d(1., 2., 3.)));
}

// Tests properties from building the bounding volume tree.
TEST_F(BVHTest, TestBuildBVTree) {
  // Since it's a binary tree with a single element at each leaf node, the tree
  // depth can be found using 2^d = num_elements. The octahedron has 8 elements
  // so we should end up with a balanced tree of depth 3 where each level's
  // volume is less than its parent's volume.
  const BvNode<SurfaceMesh<double>>& bv_tree = bvh_.root_node();
  const int num_elements = mesh_.num_elements();
  std::set<SurfaceFaceIndex> element_indices;
  std::function<void(const BvNode<SurfaceMesh<double>>&, int)> check_node;
  check_node = [&check_node, &element_indices, num_elements](
                   const BvNode<SurfaceMesh<double>>& node, int depth) {
    if (depth < 3) {
      const double node_volume = node.aabb().CalcVolume();
      const Vector3d node_bounds_upper = node.aabb().upper();
      const Vector3d node_bounds_lower = node.aabb().lower();

      auto check_child = [&check_node, &node_volume, &node_bounds_upper,
                          &node_bounds_lower,
                          &depth](const BvNode<SurfaceMesh<double>>& child) {
        // Check the volume is less than the parent.
        const double child_volume = child.aabb().CalcVolume();
        EXPECT_LT(child_volume, node_volume);
        // Check the bounds are within the parent.
        const Vector3d child_bounds_upper = child.aabb().upper();
        const Vector3d child_bounds_lower = child.aabb().lower();
        // Instead of comparing each element, we can use the coefficient-wise
        // min and max functions. If the child box is inside the parent box,
        // then the child's maximum extents must be less than or equal to the
        // parent's maximum extents. So, the smallest maximum extents across the
        // two must be equal to the child's. A similar trick works with the
        // minimum extents.
        EXPECT_TRUE(
            CompareMatrices(node_bounds_upper.cwiseMin(child_bounds_upper),
                            child_bounds_upper));
        EXPECT_TRUE(
            CompareMatrices(node_bounds_lower.cwiseMax(child_bounds_lower),
                            child_bounds_lower));
        // Check each of the branches at increasing depth.
        check_node(child, depth + 1);
      };
      check_child(node.left());
      check_child(node.right());
    } else {
      // At depth 3, we should reach the leaf node with a valid and unique
      // element instead of more branches.
      EXPECT_GE(node.element_index(), 0);
      EXPECT_LT(node.element_index(), num_elements);
      EXPECT_EQ(element_indices.count(node.element_index()), 0);
      element_indices.insert(node.element_index());
    }
  };
  check_node(bv_tree, 0);
  // Check that we found a leaf node for all elements.
  EXPECT_EQ(element_indices.size(), num_elements);
}

// Tests copy constructor.
TEST_F(BVHTest, TestCopy) {
  // Copy constructor.
  BoundingVolumeHierarchy<SurfaceMesh<double>> bvh_copy(bvh_);

  // Confirm that it's a deep copy.
  std::function<void(const BvNode<SurfaceMesh<double>>&,
                     const BvNode<SurfaceMesh<double>>&)> check_copy;
  check_copy = [&check_copy](const BvNode<SurfaceMesh<double>>& orig,
                             const BvNode<SurfaceMesh<double>>& copy) {
    EXPECT_NE(&orig, &copy);
    if (orig.is_leaf()) {
      EXPECT_EQ(orig.element_index(), copy.element_index());
    } else {
      check_copy(orig.left(), copy.left());
      check_copy(orig.right(), copy.right());
    }
  };
  check_copy(bvh_.root_node(), bvh_copy.root_node());
}

// Tests colliding while traversing through the bvh trees. We want to ensure
// that the 4 cases of branch and leaf comparison are covered, as well as the
// case of no overlap.
TEST_F(BVHTest, TestCollide) {
  // The two trees are completely separate so no bounding volumes overlap and
  // all pairs should be culled. The resulting vector should thus be empty.
  auto separate_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  RigidTransformd X_WV{Vector3d{4, 4, 4}};
  BoundingVolumeHierarchy<SurfaceMesh<double>> separate(separate_mesh);
  std::vector<std::pair<SurfaceFaceIndex, SurfaceFaceIndex>> pairs =
      bvh_.GetCollidingPairs(separate, X_WV);
  EXPECT_EQ(pairs.size(), 0);

  // Create a higher resolution mesh so we have a different number of elements
  // across the bvh trees, i.e. 8 in our coarse octahedron and 32 here. We place
  // the meshes such that they are touching at one corner, so the traversal will
  // reach end leaf-leaf cases since there are potentially colliding pairs.
  // Since the trees have different depths, the traversal will cover branch-leaf
  // cases on its way. Swapping the order then catches the opposing leaf-branch
  // cases.
  auto tangent_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 2);
  X_WV = RigidTransformd{Vector3d{3, 0, 0}};
  BoundingVolumeHierarchy<SurfaceMesh<double>> tangent(tangent_mesh);
  pairs = bvh_.GetCollidingPairs(tangent, X_WV);
  EXPECT_EQ(pairs.size(), 16);
  pairs = tangent.GetCollidingPairs(bvh_, X_WV);
  EXPECT_EQ(pairs.size(), 16);
}

// Tests colliding the bvh trees with early exit and typed transforms.
TEST_F(BVHTest, TestCollideEarlyExit) {
  // Since we're using a copy of the bvh there should be n^2 potentially
  // colliding pairs, but we should only get up to our specified limit after
  // the early exit.
  BoundingVolumeHierarchy<SurfaceMesh<double>> copy(bvh_);
  int count{0};
  int limit{1};
  // This callback should only be run as many times as the specified limit
  // before the early exit kicks in.
  auto callback = [&count, &limit](SurfaceFaceIndex a,
                                   SurfaceFaceIndex b) -> BvttCallbackResult {
    ++count;
    return count < limit ? BvttCallbackResult::Continue
                         : BvttCallbackResult::Terminate;
  };
  bvh_.Collide(copy, math::RigidTransform<AutoDiffXd>::Identity(), callback);
  EXPECT_EQ(count, 1);

  count = 0;
  limit = 5;
  bvh_.Collide(copy, math::RigidTransform<AutoDiffXd>::Identity(), callback);
  EXPECT_EQ(count, 5);
}

// Tests colliding the bvh trees with different mesh types.
TEST_F(BVHTest, TestCollideSurfaceVolume) {
  // The two octahedrons are tangentially touching along the X-axis, so there
  // should be 4 elements each that are colliding, resulting in 4^2 = 16.
  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(Ellipsoid(1.5, 2., 3.), 6);
  BoundingVolumeHierarchy<VolumeMesh<double>> tet_bvh(volume_mesh);
  auto surface_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  RigidTransformd X_WV{Vector3d{3, 0, 0}};
  BoundingVolumeHierarchy<SurfaceMesh<double>> tri_bvh(surface_mesh);

  auto pairs = tet_bvh.GetCollidingPairs(tri_bvh, X_WV);
  EXPECT_EQ(pairs.size(), 16);
}

// Tests computing the centroid of an element.
GTEST_TEST(BoundingVolumeHierarchyTest, TestComputeCentroid) {
  // Set resolution at double so that we get the coarsest mesh of 8 elements.
  auto surface_mesh =
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6);
  Vector3d centroid = BVHTester::ComputeCentroid<SurfaceMesh<double>>(
      surface_mesh, SurfaceFaceIndex(0));
  // The first face of our octahedron is a triangle with vertices at 1, 2, and
  // 3 along each respective axis, so its centroid should average out to 1/3,
  // 2/3, and 3/3.
  EXPECT_TRUE(CompareMatrices(centroid, Vector3d(1./3., 2./3., 1.)));

  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(Ellipsoid(1., 2., 3.), 6);
  centroid = BVHTester::ComputeCentroid<VolumeMesh<double>>(
      volume_mesh, VolumeElementIndex(0));
  // The first face of our octahedron is a tet with vertices at 1, 2, and 3
  // along each respective axis and the origin 0, so its centroid should
  // average out to 1/4, 2/4, and 3/4.
  EXPECT_TRUE(CompareMatrices(centroid, Vector3d(0.25, 0.5, 0.75)));
}

// Tests calculating the bounding box volume.
GTEST_TEST(AABBTest, TestVolume) {
  Aabb aabb = Aabb(Vector3d(-1, 2, 1), Vector3d(2, 0.5, 2.7));
  EXPECT_EQ(aabb.CalcVolume(), 21.6);
  Aabb zero_aabb = Aabb(Vector3d(3, -4, 1.3), Vector3d(0, 0, 0));
  EXPECT_EQ(zero_aabb.CalcVolume(), 0);
}

// Tests calculating the bounding points.
GTEST_TEST(AABBTest, TestBounds) {
  Aabb aabb = Aabb(Vector3d(-1, 2, 1), Vector3d(2, 0.5, 2.7));
  EXPECT_TRUE(CompareMatrices(aabb.upper(), Vector3d(1, 2.5, 3.7)));
  EXPECT_TRUE(CompareMatrices(aabb.lower(), Vector3d(-3, 1.5, -1.7), 1e-15));
}

// Tests whether OBBs overlap.
GTEST_TEST(AABBTest, TestObbOverlap) {
  // One box is fully contained in the other.
  Aabb a = Aabb(Vector3d(0, 0, 0), Vector3d(1, 2, 1));
  Aabb b = Aabb(Vector3d(0, 0, 0), Vector3d(0.5, 1, 0.5));
  EXPECT_TRUE(Aabb::HasOverlap(a, b, RigidTransformd::Identity()));

  // The two boxes are aligned and touching on a single face.
  b = Aabb(Vector3d(2, 0, 0), Vector3d(1, 2, 1));
  EXPECT_TRUE(Aabb::HasOverlap(a, b, RigidTransformd::Identity()));

  // Separate from the volume using center along x-axis.
  b = Aabb(Vector3d(2.1, 0, 0), Vector3d(1, 2, 1));
  EXPECT_FALSE(Aabb::HasOverlap(a, b, RigidTransformd::Identity()));

  // Separate from the volume using translation along y-axis.
  b = Aabb(Vector3d(0, 0, 0), Vector3d(1, 2, 1));
  EXPECT_FALSE(Aabb::HasOverlap(a, b, RigidTransformd{Vector3d{0, 4.1, 0}}));

  // Separate from the volume using translation along z-axis.
  b = Aabb(Vector3d(0, 0, 0), Vector3d(1, 2, 1));
  EXPECT_FALSE(Aabb::HasOverlap(a, b, RigidTransformd{Vector3d{0, 0, 3.1}}));

  // Overlapping with offset center despite translation.
  b = Aabb(Vector3d(-2, 0, 0), Vector3d(1, 2, 1));
  EXPECT_TRUE(Aabb::HasOverlap(a, b, RigidTransformd{Vector3d{2.1, 0, 0}}));

  // Separate along b's x-axis.
  b = Aabb(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{
          math::RotationMatrixd{AngleAxisd(M_PI / 4, Vector3d::UnitY())},
          Vector3d{2, 0, -2}}));
  // Separate along b's y-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{
          math::RotationMatrixd{AngleAxisd(M_PI / 4, Vector3d::UnitZ())},
          Vector3d{-2, 3, 0}}));
  // Separate along b's z-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{
          math::RotationMatrixd{AngleAxisd(M_PI / 4, Vector3d::UnitX())},
          Vector3d{0, -3, 2}}));

  // Separate along a's x-axis and b's x-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{0, -M_PI / 4, M_PI / 4},
                      Vector3d{0, -3.1, 2.2}}));
  // Separate along a's x-axis and b's y-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{M_PI / 4, 0, M_PI / 4},
                      Vector3d{0, -3.1, 2.2}}));
  // Separate along a's x-axis and b's z-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{0, M_PI / 4, M_PI / 4},
                      Vector3d{0, -3.1, 2.2}}));
  // Separate along a's y-axis and b's x-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{M_PI / 4, M_PI / 4, 0},
                      Vector3d{2.2, 0, 2.2}}));
  // Separate along a's y-axis and b's y-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{M_PI / 4, 0, M_PI / 4},
                      Vector3d{2.2, 0, 2.2}}));
  // Separate along a's y-axis and b's z-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{0, -M_PI / 4, M_PI / 4},
                      Vector3d{2.2, 0, 2.2}}));
  // Separate along a's z-axis and b's x-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{M_PI / 4, 0, -M_PI / 4},
                      Vector3d{2.2, 3.2, 0}}));
  // Separate along a's z-axis and b's y-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{0, -M_PI / 4, M_PI / 4},
                      Vector3d{2.2, 3.2, 0}}));
  // Separate along a's z-axis and b's z-axis.
  EXPECT_FALSE(Aabb::HasOverlap(
      a, b,
      RigidTransformd{math::RollPitchYawd{M_PI / 4, M_PI / 4, 0},
                      Vector3d{2.2, 3.2, 0}}));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
