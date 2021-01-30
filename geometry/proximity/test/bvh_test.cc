#include "drake/geometry/proximity/bvh.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/bounding_volume_hierarchy.h"
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

// Friend class for accessing Bvh's protected/private functionality.
class BvhTester {
 public:
  BvhTester() = delete;

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

  // Returns true iff the oriented bounding box contains all the specified
  // elements in the mesh.
  template <typename MeshType>
  static bool Contain(
      const Obb& obb_M, const MeshType& mesh_M,
      const typename std::vector<typename Bvh<MeshType>::CentroidPair>::iterator
          start,
      const typename std::vector<typename Bvh<MeshType>::CentroidPair>::iterator
          end) {
    const RigidTransformd& X_MB = obb_M.pose();
    const RigidTransformd X_BM = X_MB.inverse();
    for (auto it = start; it != end; ++it) {
      const auto& element = mesh_M.element(it->first);
      for (int i=0; i < MeshType::kVertexPerElement; ++i) {
        const Vector3d p_MV = mesh_M.vertex(element.vertex(i)).r_MV();
        const Vector3d p_BV = X_BM * p_MV;
        if ((p_BV.array() > obb_M.half_width().array()).any()) {
          return false;
        }
        if ((p_BV.array() < -obb_M.half_width().array()).any()) {
          return false;
        }
      }
    }
    return true;
  }
};

namespace {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

// Since we are deprecating BoundingVolumeHierarchy by aliasing it as Bvh,
// here we test an instantiation of the alias. We should remove this test at
// the end of deprecation period.
GTEST_TEST(BoundingVolumeHierarchyTest, TestInstantiation) {
  const SurfaceMesh<double> mesh =
      MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  const BoundingVolumeHierarchy<SurfaceMesh<double>> bvh(mesh);
}

#pragma GCC diagnostic pop  // pop "-Wdeprecated-declarations"

GTEST_TEST(BvNodeTest, TestEqualLeaf) {
  // Bounding volume is not used in EqualLeaf. It is used in EqualTrees.
  // We can use any Obb for this test.
  Obb bv(RigidTransformd::Identity(), Vector3d::Zero());

  BvNode<SurfaceMesh<double>> leaf_of_one_element(bv,
                                                  {1, {SurfaceFaceIndex(0)}});
  // Tests reflexive property: equal to itself.
  EXPECT_TRUE(leaf_of_one_element.EqualLeaf(leaf_of_one_element));

  // Unequal number of elements.
  BvNode<SurfaceMesh<double>> leaf_of_two_elements(
      bv, {2, {SurfaceFaceIndex(0), SurfaceFaceIndex(1)}});
  EXPECT_FALSE(leaf_of_one_element.EqualLeaf(leaf_of_two_elements));

  // Second element is different.
  BvNode<SurfaceMesh<double>> leaf_with_a_different_element(
      bv, {2, {SurfaceFaceIndex(0), SurfaceFaceIndex(2)}});
  EXPECT_FALSE(leaf_of_two_elements.EqualLeaf(leaf_with_a_different_element));

  // All elements are the same.
  BvNode<SurfaceMesh<double>> leaf_with_same_two_elements(
      bv, {2, {SurfaceFaceIndex(0), SurfaceFaceIndex(1)}});
  EXPECT_TRUE(leaf_of_two_elements.EqualLeaf(leaf_with_same_two_elements));
}

// @tparam BvNodeType is either BvNode<SurfaceMesh<double>> or
//                    BvNode<VolumeMesh<double>>.
template <class BvNodeType>
int CountAllNodes(const BvNodeType& root) {
  if (root.is_leaf()) {
    return 1;
  } else {
    int left_result = CountAllNodes(root.left());
    int right_result = CountAllNodes(root.right());
    return 1 + left_result + right_result;
  }
}

// @tparam BvNodeType is either BvNode<SurfaceMesh<double>> or
//                    BvNode<VolumeMesh<double>>.
template <class BvNodeType>
int CountLeafNodes(const BvNodeType& root) {
  if (root.is_leaf()) {
    return 1;
  } else {
    int left_result = CountLeafNodes(root.left());
    int right_result = CountLeafNodes(root.right());
    return left_result + right_result;
  }
}

// The height of a tree is the number of edges in a longest possible path,
// going away from the root, that starts at the root and ends at a leaf.
// (https://en.m.wikipedia.org/wiki/Glossary_of_graph_theory_terms#height)
//
// @tparam BvNodeType is either BvNode<SurfaceMesh<double>> or
//                    BvNode<VolumeMesh<double>>.
template <class BvNodeType>
int ComputeHeight(const BvNodeType& root) {
  if (root.is_leaf()) {
    return 0;
  }
  return 1 + std::max(ComputeHeight(root.left()),
                      ComputeHeight(root.right()));
}

// Tests that the right element indices are used in the calculation. We can
// use a "toy" example because we're not testing the robustness of the OBB
// calculations (those are tested in obb_test.cc), merely the propagation of
// the correct parameters.
GTEST_TEST(BvhTestAPI, TestComputeBoundingVolume) {
  // A mesh of two triangles that are well separated by X-Y plane.
  const SurfaceMesh<double> mesh(
      {SurfaceFace(SurfaceVertexIndex(0), SurfaceVertexIndex(1),
                   SurfaceVertexIndex(2)),
       SurfaceFace(SurfaceVertexIndex(3), SurfaceVertexIndex(4),
                   SurfaceVertexIndex(5))},
      {SurfaceVertex<double>(Vector3d(0, 0, 1)),
       SurfaceVertex<double>(Vector3d(1, 0, 1)),
       SurfaceVertex<double>(Vector3d(0, 1, 1)),
       SurfaceVertex<double>(Vector3d(0, 0, -1)),
       SurfaceVertex<double>(Vector3d(1, 0, -1)),
       SurfaceVertex<double>(Vector3d(0, 1, -1))});

  using FaceCentroidPair = std::pair<SurfaceFaceIndex, Vector3d>;
  // The positions of centroids are not relevant to this test.
  std::vector<FaceCentroidPair> upper{{SurfaceFaceIndex(0), Vector3d()}};
  std::vector<FaceCentroidPair> lower{{SurfaceFaceIndex(1), Vector3d()}};

  Obb bv = BvhTester::ComputeBoundingVolume(mesh, upper.begin(), upper.end());
  EXPECT_TRUE(BvhTester::Contain(bv, mesh, upper.begin(), upper.end()));
  EXPECT_FALSE(BvhTester::Contain(bv, mesh, lower.begin(), lower.end()));
}

// Test fixture class for testing Bvh. Uses a coarse sphere, i.e. an
// octahedron, as the underlying mesh. We expect the resultant BVH to be a
// perfect binary tree as tested in TestBuildBvTree.
class BvhTest : public ::testing::Test {
 public:
  BvhTest()
      : ::testing::Test(),
        mesh_(MakeSphereSurfaceMesh<double>(Sphere(1.5), 3)),
        bvh_(Bvh<SurfaceMesh<double>>(mesh_)) {}

 protected:
  SurfaceMesh<double> mesh_;
  Bvh<SurfaceMesh<double>> bvh_;
};

// Tests properties from building the bounding volume tree.
TEST_F(BvhTest, TestBuildBvTree) {
  // Verify that it is a perfect binary tree with two mesh elements per leaf,
  // so the tree height h must satisfy 2^h = num_elements/2. The octahedron
  // has 8 elements, so we should end up with a tree of height 2 where each
  // level's volume is less than its parent's volume. The following picture
  // shows the structure of the expected perfect binary tree. The number
  // shown in each node indicates the number of mesh elements that belong to
  // the subtree rooted at that node.
  //
  //                      8
  //                   ／   ＼
  //                 4        4
  //               ／＼       ／＼
  //            2     2     2     2
  //
  EXPECT_EQ(CountAllNodes(bvh_.root_node()), 7);
  EXPECT_EQ(CountLeafNodes(bvh_.root_node()), 4);
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
      EXPECT_EQ(node.num_element_indices(), 2);
      for (int i = 0; i < node.num_element_indices(); ++i) {
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
TEST_F(BvhTest, TestCopy) {
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
TEST_F(BvhTest, TestCollide) {
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
  // L = 2^H. We expect every leaf to have two triangles which leads to
  // 32/2 = 16 leaf nodes. That yields an expected height of 4. As an extra
  // check, we also confirm the number of nodes 2^(4+1)-1 = 31.
  ASSERT_EQ(ComputeHeight(tangent.root_node()), 4);
  ASSERT_EQ(CountLeafNodes(tangent.root_node()), 16);
  ASSERT_EQ(CountAllNodes(tangent.root_node()), 31);

  // The results of 100 collision candidates is an empirical observation only.
  // The significance here is that we get the same number in both cases. We
  // assume that equal number implies same contents.
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
TEST_F(BvhTest, TestCollideEarlyExit) {
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
TEST_F(BvhTest, TestCollideSurfaceVolume) {
  // The two octahedrons are tangentially touching along the X-axis, so there
  // should be 4 elements each that are colliding, resulting in 4^2 = 16.
  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1.5, 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  ASSERT_EQ(volume_mesh.num_elements(), 8);
  Bvh<VolumeMesh<double>> tet_bvh(volume_mesh);
  // Confirm this tree structure of tet_bvh.
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
  ASSERT_EQ(CountLeafNodes(tet_bvh.root_node()), 8);
  ASSERT_EQ(CountAllNodes(tet_bvh.root_node()), 15);

  auto surface_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  ASSERT_EQ(surface_mesh.num_elements(), 8);
  RigidTransformd X_WV{Vector3d{3, 0, 0}};
  Bvh<SurfaceMesh<double>> tri_bvh(surface_mesh);
  // Confirm this tree structure of tri_bvh.
  //
  //                      8
  //                   ／   ＼
  //                 4        4
  //               ／＼       ／＼
  //            2     2     2     2
  //
  ASSERT_EQ(ComputeHeight(tri_bvh.root_node()), 2);
  ASSERT_EQ(CountLeafNodes(tri_bvh.root_node()), 4);
  ASSERT_EQ(CountAllNodes(tri_bvh.root_node()), 7);

  // The two octahedrons are tangentially touching along the X-axis. There
  // should be 4 Obbs from each Bvh that are colliding, resulting in 4x4 = 16
  // pairs of leaves. Each pair of leaves give two tetrahedron-triangle pairs
  // because tet_bvh's leaf has one element and tri_bvh's leaf has two
  // elements as verified above. Totally there are 16x2 = 32 candidate pairs.
  auto pairs = tet_bvh.GetCollisionCandidates(tri_bvh, X_WV);
  EXPECT_EQ(pairs.size(), 32);
}

// Tests colliding the bvh tree with a primitive object. We use a plane as a
// representative primitive.
TEST_F(BvhTest, TestCollidePrimitive) {
  // Use Y-Z plane of frame P, i.e., the plane is perpendicular to Px.
  // Using such a trivial primitive is fine because we're not testing the
  // robustness of primitive-obb algorithms, but the Bvh algorithm that
  // iterates through them. We rely on the obb-primitive tests to handle the
  // robustness.
  Plane<double> plane_P{Vector3d::UnitX(), Vector3d::Zero()};

  // All collision candidates.
  {
    std::vector<SurfaceFaceIndex> candidates;
    // We pose the plane to pass through a plane of symmetry of the octahedron,
    // so we should get all 8 triangles.
    bvh_.Collide(plane_P, RigidTransformd::Identity(),
                 [&candidates](SurfaceFaceIndex f) -> BvttCallbackResult {
                   candidates.push_back(f);
                   return BvttCallbackResult::Continue;
                 });
    int num_candidates = candidates.size();
    EXPECT_EQ(num_candidates, 8);
  }

  // First collision candidate then terminate early.
  {
    std::vector<SurfaceFaceIndex> candidates;
    bvh_.Collide(plane_P, RigidTransformd::Identity(),
                 [&candidates](SurfaceFaceIndex f) -> BvttCallbackResult {
                   candidates.push_back(f);
                   return BvttCallbackResult::Terminate;
                 });
    int num_candidates = candidates.size();
    EXPECT_EQ(num_candidates, 1);
  }

  // No collision.
  {
    std::vector<SurfaceFaceIndex> candidates;
    // We pose the plane far from the mesh.
    bvh_.Collide(plane_P, RigidTransformd(Vector3d(10, 0, 0)),
                 [&candidates](SurfaceFaceIndex f) -> BvttCallbackResult {
                   candidates.push_back(f);
                   return BvttCallbackResult::Continue;
                 });
    int num_candidates = candidates.size();
    EXPECT_EQ(num_candidates, 0);
  }
}

// Tests computing the centroid of an element.
GTEST_TEST(BoundingVolumeHierarchyTest, TestComputeCentroid) {
  // Set resolution at double so that we get the coarsest mesh of 8 elements.
  auto surface_mesh =
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6);
  Vector3d centroid = BvhTester::ComputeCentroid<SurfaceMesh<double>>(
      surface_mesh, SurfaceFaceIndex(0));
  // The first face of our octahedron is a triangle with vertices at 1, 2, and
  // 3 along each respective axis, so its centroid should average out to 1/3,
  // 2/3, and 3/3.
  EXPECT_TRUE(CompareMatrices(centroid, Vector3d(1. / 3., 2. / 3., 1.)));

  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1., 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  centroid = BvhTester::ComputeCentroid<VolumeMesh<double>>(
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
  // In order to require multiple recursive calls, both trees must have
  // non-zero height, *and* the root nodes must have "equal" bounding volumes.
  // These tests confirm those preconditions.
  ASSERT_GT(ComputeHeight(bvh_ellipsoid.root_node()), 0);
  ASSERT_GT(ComputeHeight(bvh_ellipsoid_too.root_node()), 0);
  ASSERT_TRUE(
      bvh_ellipsoid.root_node().bv().Equal(bvh_ellipsoid_too.root_node().bv()));

  EXPECT_TRUE(bvh_ellipsoid.Equal(bvh_ellipsoid_too));

  // Tests reflexive property: equal to itself.
  EXPECT_TRUE(bvh_ellipsoid.Equal(bvh_ellipsoid));
}

// Simply confirms that an Obb can be built from an autodiff mesh. We apply a
// limited smoke test to indicate success -- the bounding volume of the root
// node is the same as if the mesh were double-valued.
GTEST_TEST(BoundingVolumeHierarchyTest, BvhFromAutodiffmesh) {
  SurfaceMesh<AutoDiffXd> mesh_ad =
      MakeSphereSurfaceMesh<AutoDiffXd>(Sphere(1.5), 3);
  Bvh<SurfaceMesh<AutoDiffXd>> bvh_ad(mesh_ad);
  SurfaceMesh<double> mesh_d =
      MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  Bvh<SurfaceMesh<double>> bvh_d(mesh_d);
  EXPECT_TRUE(bvh_ad.root_node().bv().Equal(bvh_d.root_node().bv()));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
