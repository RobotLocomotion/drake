#include "drake/geometry/proximity/bvh.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

// Friend class for accessing AAbb's protected/private functionality.
class ObbTester : public ::testing::Test {
 public:
  static constexpr double kTolerance = Obb::kTolerance;
};

// Friend class for accessing Bvh's protected/private functionality.
class BVHTester {
 public:
  BVHTester() = delete;

  template <class MeshType>
  static Vector3d ComputeCentroid(const MeshType& mesh,
                                  const typename MeshType::ElementIndex i) {
    return Bvh<MeshType>::ComputeCentroid(mesh, i);
  }

  template <class MeshType>
  static Obb ComputeBoundingVolume(
      const MeshType& mesh,
      const typename std::vector<typename Bvh<MeshType>::CentroidPair>::iterator
          start,
      const typename std::vector<typename Bvh<MeshType>::CentroidPair>::iterator
          end) {
    return Bvh<MeshType>::ComputeBoundingVolume(mesh, start, end);
  }
};

namespace {

template <class BvNodeType>
int CountNumNodes(const BvNodeType& root) {
  if (root.is_leaf()) {
    return 1;
  } else {
    int left_result = CountNumNodes(root.left());
    int right_result = CountNumNodes(root.right());
    return 1 + left_result + right_result;
  }
}

template <class BvNodeType>
int CountNumLeaves(const BvNodeType& root) {
  if (root.is_leaf()) {
    return 1;
  } else {
    int left_result = CountNumLeaves(root.left());
    int right_result = CountNumLeaves(root.right());
    return left_result + right_result;
  }
}

// The height of a tree is the number of edges in a longest possible path,
// going away from the root, that starts at the root and ends at a leaf.
// (https://en.m.wikipedia.org/wiki/Glossary_of_graph_theory_terms#height)
template <class BvNodeType>
int ComputeHeight(const BvNodeType& root) {
  if (root.is_leaf()) {
    return 0;
  }
  return 1 + std::max(ComputeHeight(root.left()),
                      ComputeHeight(root.right()));
}

// Test fixture class for testing Bvh. Uses a coarse
// sphere, i.e. an octahedron, as the underlying mesh.
class BVHTest : public ::testing::Test {
 public:
  BVHTest()
      : ::testing::Test(),
        mesh_(MakeSphereSurfaceMesh<double>(Sphere(1.5), 3)),
        bvh_(Bvh<SurfaceMesh<double>>(mesh_)) {}

 protected:
  SurfaceMesh<double> mesh_;
  Bvh<SurfaceMesh<double>> bvh_;
};

// Smoke tests for computing the bounding volume of elements. The correctness
// of ComputeBoundingVolume() depends on ObbMaker, which is tested in
// obb_test.cc.
TEST_F(BVHTest, TestComputeBoundingVolume) {
  std::vector<std::pair<SurfaceFaceIndex, Vector3d>> tri_centroids;
  // Add all the elements from the octahedron to test multiple elements. The
  // centroid values are irrelevant for this test.
  for (SurfaceFaceIndex i(0); i < mesh_.num_elements(); ++i) {
    tri_centroids.emplace_back(i, Vector3d(0.5, 0.5, 0.5));
  }
  Obb bv = BVHTester::ComputeBoundingVolume<SurfaceMesh<double>>(
      mesh_, tri_centroids.begin(), tri_centroids.end());
  // It is not easy to predict the oriented bounding box, so we only check its
  // volume. The threshold was set empirically. It is smaller than the
  // trivial upper bound 27.0, which is the volume of a cube bounding the
  // sphere. In the future, we can decrease this threshold if we improve the
  // code.
  EXPECT_NEAR(bv.CalcVolume(), 26.996, 0.001);

  // Test with a volume mesh. As above, the centroids are still irrelevant.
  std::vector<std::pair<VolumeElementIndex, Vector3d>> tet_centroids;
  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1., 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  for (VolumeElementIndex i(0); i < volume_mesh.num_elements(); ++i) {
    tet_centroids.emplace_back(i, Vector3d(0.5, 0.5, 0.5));
  }
  bv = BVHTester::ComputeBoundingVolume<VolumeMesh<double>>(
      volume_mesh, tet_centroids.begin(), tet_centroids.end());
  // It is not easy to predict the oriented bounding box, so we only check its
  // volume. The threshold was set empirically. It is smaller than the
  // trivial upper bound 48.0, which is the volume of the axis-aligned box
  // bounding the ellipsoid. In the future, we can decrease this threshold if
  // we improve the code.
  EXPECT_NEAR(bv.CalcVolume(), 41.316, 0.001);
}

// Tests properties from building the bounding volume tree.
TEST_F(BVHTest, TestBuildBVTree) {
  // Verify that it is a perfect binary tree with two mesh elements at each
  // leaf, so the tree height h must satisfy 2^h = num_elements/2. The
  // octahedron has 8 elements, so we should end up with a tree of height 2
  // where each level's volume is less than its parent's volume.
  //
  //                      8
  //                   ／   ＼
  //                 4        4
  //               ／＼       ／＼
  //            2     2     2     2
  //
  EXPECT_EQ(CountNumNodes(bvh_.root_node()), 7);
  EXPECT_EQ(CountNumLeaves(bvh_.root_node()), 4);
  const int tree_height = ComputeHeight(bvh_.root_node());
  EXPECT_EQ(tree_height, 2);

  const int num_elements = mesh_.num_elements();
  std::set<SurfaceFaceIndex> element_indices;
  std::function<void(const BvNode<SurfaceMesh<double>>&, int)> check_node;
  check_node = [&check_node, &element_indices, num_elements, tree_height](
                   const BvNode<SurfaceMesh<double>>& node, int depth) {
    if (depth < tree_height) {
      EXPECT_FALSE(node.is_leaf());
      const double node_volume = node.bv().CalcVolume();
      EXPECT_LT(node.left().bv().CalcVolume(), node_volume);
      EXPECT_LT(node.right().bv().CalcVolume(), node_volume);
      // Check each of the branches at increasing depth.
      check_node(node.left(), depth + 1);
      check_node(node.right(), depth + 1);
    } else {
      // We should reach the leaf node with valid and unique elements instead
      // of more branches.
      EXPECT_TRUE(node.is_leaf());
      for (int i = 0; i < node.num_element_index(); ++i) {
        EXPECT_GE(node.element_index(i), 0);
        EXPECT_LT(node.element_index(i), num_elements);
        EXPECT_EQ(element_indices.count(node.element_index(i)), 0);
        element_indices.insert(node.element_index(i));
      }
    }
  };
  check_node(bvh_.root_node(), 0);
  // Check that we found a leaf node for all elements.
  EXPECT_EQ(element_indices.size(), num_elements);
}

// Tests copy constructor.
TEST_F(BVHTest, TestCopy) {
  // Copy constructor.
  Bvh<SurfaceMesh<double>> bvh_copy(bvh_);

  // Confirm that it's a deep copy.
  std::function<void(const BvNode<SurfaceMesh<double>>&,
                     const BvNode<SurfaceMesh<double>>&)>
      check_copy;
  check_copy = [&check_copy](const BvNode<SurfaceMesh<double>>& orig,
                             const BvNode<SurfaceMesh<double>>& copy) {
    EXPECT_NE(&orig, &copy);
    if (orig.is_leaf()) {
      ASSERT_TRUE(copy.is_leaf());
      EXPECT_TRUE(orig.EqualLeaf(copy));
    } else {
      check_copy(orig.left(), copy.left());
      check_copy(orig.right(), copy.right());
    }
  };
  check_copy(bvh_.root_node(), bvh_copy.root_node());
}

// Tests colliding while traversing through the bvh trees. We want to ensure
// that the case of no overlap is covered as well as the 4 cases of branch and
// leaf comparisons, i.e:
//  1. branch : branch
//  2. branch : leaf
//  3. leaf : branch
//  4. leaf : leaf
TEST_F(BVHTest, TestCollide) {
  // The two trees are completely separate so no bounding volumes overlap and
  // all pairs should be culled. The resulting vector should thus be empty.
  auto separate_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  RigidTransformd X_WV{Vector3d{4, 4, 4}};
  Bvh<SurfaceMesh<double>> separate(separate_mesh);
  std::vector<std::pair<SurfaceFaceIndex, SurfaceFaceIndex>> pairs =
      bvh_.GetCollisionCandidates(separate, X_WV);
  EXPECT_EQ(pairs.size(), 0);

  // Create a higher resolution mesh so we have a different number of elements
  // across the bvh trees, i.e. 8 in our coarse octahedron and 32 here. We
  // place the meshes such that they are touching at one corner, so the
  // traversal will reach end leaf-leaf cases (4.) since there are potentially
  // colliding pairs. Since the two perfect binary trees have different
  // heights, the traversal will cover branch-leaf cases (2.) on its way.
  // Swapping the order then catches the opposing leaf-branch cases (3.).
  // Since the trees have multiple depths, then at higher levels, for example
  // at the root, the branch-branch cases (1.) will be covered.
  auto tangent_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 2);
  ASSERT_EQ(tangent_mesh.num_elements(), 32);
  X_WV = RigidTransformd{Vector3d{3, 0, 0}};
  Bvh<SurfaceMesh<double>> tangent(tangent_mesh);
  // Reality check that the tree of `tangent` is a perfect binary tree. A
  // perfect binary tree with height H has number of leaves L satisfying
  // L = 2^H. The `tangent` tree has height 4 and 2^4 = 16 leaves, which is
  // half the number of triangles in the mesh. As an extra check, we also
  // confirm the number of nodes 2^(4+1)-1 = 31.
  ASSERT_EQ(ComputeHeight(tangent.root_node()), 4);
  ASSERT_EQ(CountNumLeaves(tangent.root_node()), 16);
  ASSERT_EQ(CountNumNodes(tangent.root_node()), 31);

  // With extra inspection code (not shown here), we found that there were 25
  // pairs of leaves from the two trees with overlapping bounding volumes.
  // Each pair of leaves gave 4 pairs of triangles. Totally there were
  // 25 x 4 = 100 candidate pairs of triangles. The exact number of
  // candidates is less relevant to this test, so we did not investigate
  // further.
  pairs = bvh_.GetCollisionCandidates(tangent, X_WV);
  EXPECT_EQ(pairs.size(), 100);
  pairs = tangent.GetCollisionCandidates(bvh_, X_WV);
  EXPECT_EQ(pairs.size(), 100);
}

// Tests colliding while traversing through the bvh trees but with early exit.
// We want to ensure that the trees are not fully traversed. One way to test
// this is to count towards a limit as the condition for the exit. If we
// specify a limit that is less than the number of potentially colliding pairs,
// we can verify that the traversal has exited since our result can be no more
// than this limit.
TEST_F(BVHTest, TestCollideEarlyExit) {
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
  // Since we're colliding bvh against itself there should be up to n^2
  // potentially colliding pairs, but we max out at our limit of 1.
  bvh_.Collide(bvh_, math::RigidTransformd::Identity(), callback);
  EXPECT_EQ(count, 1);

  count = 0;
  limit = 5;
  // Updating the limit to 5 should get further in the traversal with a result
  // of 5 pairs.
  bvh_.Collide(bvh_, math::RigidTransformd::Identity(), callback);
  EXPECT_EQ(count, 5);
}

// Tests colliding the bvh trees with different mesh types, i.e. mixing tris
// and tets by colliding surface and volume meshes.
TEST_F(BVHTest, TestCollideSurfaceVolume) {
  // The two octahedrons are tangentially touching along the X-axis, so there
  // should be 4 elements each that are colliding, resulting in 4^2 = 16.
  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1.5, 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  ASSERT_EQ(volume_mesh.num_elements(), 8);
  Bvh<VolumeMesh<double>> tet_bvh(volume_mesh);
  // Confirm this tree structure of tet_bvh:
  //
  //                      8
  //                   ／   ＼
  //                 4        4
  //               ／＼       ／＼
  //            2     2     2     2
  //          ／＼   ／＼   ／＼   ／＼
  //         1  1   1  1  1  1  1  1
  //
  ASSERT_EQ(ComputeHeight(tet_bvh.root_node()), 3);
  ASSERT_EQ(CountNumLeaves(tet_bvh.root_node()), 8);
  ASSERT_EQ(CountNumNodes(tet_bvh.root_node()), 15);

  auto surface_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  ASSERT_EQ(surface_mesh.num_elements(), 8);
  RigidTransformd X_WV{Vector3d{3, 0, 0}};
  Bvh<SurfaceMesh<double>> tri_bvh(surface_mesh);
  // Confirm this tree structure of tri_bvh:
  //
  //                      8
  //                   ／   ＼
  //                 4        4
  //               ／＼       ／＼
  //            2     2     2     2
  //
  ASSERT_EQ(ComputeHeight(tri_bvh.root_node()), 2);
  ASSERT_EQ(CountNumLeaves(tri_bvh.root_node()), 4);
  ASSERT_EQ(CountNumNodes(tri_bvh.root_node()), 7);

  // The two octahedrons are tangentially touching along the X-axis. There
  // should be 4 Obbs from each Bvh that are colliding, resulting in 4x4 = 16
  // pairs of leaves. Each pair of leaves give two tetrahedron-triangle pairs
  // because tet_bvh's leaf has one element and tri_bvh's leaf has two
  // elements as verified above. Totally there are 16x2 = 32 candidate pairs.
  auto pairs = tet_bvh.GetCollisionCandidates(tri_bvh, X_WV);
  EXPECT_EQ(pairs.size(), 32);
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
  EXPECT_TRUE(CompareMatrices(centroid, Vector3d(1. / 3., 2. / 3., 1.)));

  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1., 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  centroid = BVHTester::ComputeCentroid<VolumeMesh<double>>(
      volume_mesh, VolumeElementIndex(0));
  // The first face of our octahedron is a tet with vertices at 1, 2, and 3
  // along each respective axis and the origin 0, so its centroid should
  // average out to 1/4, 2/4, and 3/4.
  EXPECT_TRUE(CompareMatrices(centroid, Vector3d(0.25, 0.5, 0.75)));
}

GTEST_TEST(BoundingVolumeHierarchyTest, TestEqual) {
  // Tests that two BVHs with the same tree structure but different
  // bounding volumes are not equal. Each tree has only one node.
  {
    const VolumeMesh<double> smaller_tetrahedron(
        std::vector<VolumeElement>{
            VolumeElement(VolumeVertexIndex(0), VolumeVertexIndex(1),
                          VolumeVertexIndex(2), VolumeVertexIndex(3))},
        std::vector<VolumeVertex<double>>{
            VolumeVertex<double>(Vector3d::Zero()),
            VolumeVertex<double>(Vector3d::UnitX()),
            VolumeVertex<double>(Vector3d::UnitY()),
            VolumeVertex<double>(Vector3d::UnitZ())});
    const Bvh<VolumeMesh<double>> smaller(smaller_tetrahedron);
    const VolumeMesh<double> bigger_tetrahedron(
        std::vector<VolumeElement>{
            VolumeElement(VolumeVertexIndex(0), VolumeVertexIndex(1),
                          VolumeVertexIndex(2), VolumeVertexIndex(3))},
        std::vector<VolumeVertex<double>>{
            VolumeVertex<double>(Vector3d::Zero()),
            VolumeVertex<double>(2. * Vector3d::UnitX()),
            VolumeVertex<double>(2. * Vector3d::UnitY()),
            VolumeVertex<double>(2. * Vector3d::UnitZ())});
    const Bvh<VolumeMesh<double>> bigger(bigger_tetrahedron);
    // Verify that each tree has the same structure. Each has only one node.
    ASSERT_TRUE(smaller.root_node().is_leaf());
    ASSERT_TRUE(bigger.root_node().is_leaf());
    // Verify that the two trees have different bounding volumes. Due to the
    // numerical method in computing Obb, we cannot check the bounding volume
    // of each tree easily. Instead we only assert that they are not equal.
    ASSERT_FALSE(smaller.root_node().bv().Equal(bigger.root_node().bv()));

    EXPECT_FALSE(smaller.Equal(bigger));
    EXPECT_FALSE(bigger.Equal(smaller));
  }

  // Tests that two BVHs are not equal due to different tree structures even
  // though every node has the same bounding volume.
  {
    const std::vector<VolumeVertex<double>> vertices{
        VolumeVertex<double>(Vector3d::Zero()),
        VolumeVertex<double>(Vector3d::UnitX()),
        VolumeVertex<double>(Vector3d::UnitY()),
        VolumeVertex<double>(Vector3d::UnitZ())};
    const VolumeElement element(VolumeVertexIndex(0), VolumeVertexIndex(1),
                                VolumeVertexIndex(2), VolumeVertexIndex(3));
    const VolumeMesh<double> one_tetrahedron(
        std::vector<VolumeElement>{element},
        std::vector<VolumeVertex<double>>(vertices));
    const Bvh<VolumeMesh<double>> one_node(one_tetrahedron);
    const VolumeMesh<double> duplicated_tetrahedra(
        // Both elements are the same tetrahedron, so every Bvh node will have
        // the same bounding volume.
        std::vector<VolumeElement>{element, element},
        std::vector<VolumeVertex<double>>(vertices));
    const Bvh<VolumeMesh<double>> three_nodes(duplicated_tetrahedra);
    // Verify that the two trees have different structures.
    ASSERT_TRUE(one_node.root_node().is_leaf());
    ASSERT_FALSE(three_nodes.root_node().is_leaf());
    ASSERT_TRUE(three_nodes.root_node().left().is_leaf());
    ASSERT_TRUE(three_nodes.root_node().right().is_leaf());
    // Verify that every node of the two trees has the same bounding volume.
    const Obb expect_bv = one_node.root_node().bv();
    ASSERT_TRUE(three_nodes.root_node().bv().Equal(expect_bv));
    ASSERT_TRUE(three_nodes.root_node().left().bv().Equal(expect_bv));
    ASSERT_TRUE(three_nodes.root_node().right().bv().Equal(expect_bv));

    EXPECT_FALSE(one_node.Equal(three_nodes));
    EXPECT_FALSE(three_nodes.Equal(one_node));
  }

  // Tests equality of BVH's that require multiple recursive calls in
  // traversing the trees. The mesh has enough triangles for the tree to have
  // enough depth.
  const SurfaceMesh<double> mesh_ellipsoid =
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6.);
  ASSERT_EQ(8, mesh_ellipsoid.num_faces());
  const Bvh<SurfaceMesh<double>> bvh_ellipsoid(mesh_ellipsoid);
  const Bvh<SurfaceMesh<double>> bvh_ellipsoid_too(mesh_ellipsoid);
  // Reality check that it has multiple depths before testing Equal().
  ASSERT_GT(ComputeHeight(bvh_ellipsoid.root_node()), 0);
  EXPECT_TRUE(bvh_ellipsoid.Equal(bvh_ellipsoid_too));

  // Tests reflexive property: equal to itself.
  EXPECT_TRUE(bvh_ellipsoid.Equal(bvh_ellipsoid));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
