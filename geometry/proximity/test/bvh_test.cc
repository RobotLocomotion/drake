#include "drake/geometry/proximity/bvh.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
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

  template <class BvType, class MeshType>
  static Vector3d ComputeCentroid(const MeshType& mesh, int i) {
    return Bvh<BvType, MeshType>::ComputeCentroid(mesh, i);
  }

  template <class BvType, class MeshType>
  static BvType ComputeBoundingVolume(
      const MeshType& mesh,
      const typename std::vector<
          typename Bvh<BvType, MeshType>::CentroidPair>::iterator start,
      const typename std::vector<
          typename Bvh<BvType, MeshType>::CentroidPair>::iterator end) {
    return Bvh<BvType, MeshType>::ComputeBoundingVolume(mesh, start, end);
  }

  // Returns true iff the oriented bounding box contains all the specified
  // elements in the mesh.
  template <class BvType, typename MeshType>
  static bool Contain(
      const BvType& bv_M, const MeshType& mesh_M,
      const typename std::vector<
          typename Bvh<BvType, MeshType>::CentroidPair>::iterator start,
      const typename std::vector<
          typename Bvh<BvType, MeshType>::CentroidPair>::iterator end) {
    const RigidTransformd& X_MB = bv_M.pose();
    const RigidTransformd X_BM = X_MB.inverse();
    for (auto it = start; it != end; ++it) {
      const auto& element = mesh_M.element(it->first);
      for (int i=0; i < MeshType::kVertexPerElement; ++i) {
        const Vector3d p_MV = mesh_M.vertex(element.vertex(i));
        const Vector3d p_BV = X_BM * p_MV;
        if ((p_BV.array() > bv_M.half_width().array()).any()) {
          return false;
        }
        if ((p_BV.array() < -bv_M.half_width().array()).any()) {
          return false;
        }
      }
    }
    return true;
  }
};

namespace {

GTEST_TEST(BvNodeTest, TestEqualLeaf) {
  // Bounding volume is not used in EqualLeaf. It is used in EqualTrees.
  // We can use any Obb for this test.
  Obb bv(RigidTransformd::Identity(), Vector3d::Zero());

  BvNode<Obb, TriangleSurfaceMesh<double>> leaf_of_one_element(bv, {1, {0}});
  // Tests reflexive property: equal to itself.
  EXPECT_TRUE(leaf_of_one_element.EqualLeaf(leaf_of_one_element));

  // Unequal number of elements.
  BvNode<Obb, TriangleSurfaceMesh<double>> leaf_of_two_elements(bv,
                                                                {2, {0, 1}});
  EXPECT_FALSE(leaf_of_one_element.EqualLeaf(leaf_of_two_elements));

  // Second element is different.
  BvNode<Obb, TriangleSurfaceMesh<double>> leaf_with_a_different_element(bv,
                                                                 {2, {0, 2}});
  EXPECT_FALSE(leaf_of_two_elements.EqualLeaf(leaf_with_a_different_element));

  // All elements are the same.
  BvNode<Obb, TriangleSurfaceMesh<double>> leaf_with_same_two_elements(
      bv, {2, {0, 1}});
  EXPECT_TRUE(leaf_of_two_elements.EqualLeaf(leaf_with_same_two_elements));

  // Explicitly test that we can compare leaves for meshes with different
  // declared scalar types.
  BvNode<Obb, TriangleSurfaceMesh<AutoDiffXd>> leaf_of_one_ad_element(bv,
                                                                      {1, {0}});
  EXPECT_TRUE(leaf_of_one_element.EqualLeaf(leaf_of_one_ad_element));

  // In fact, the EqualLeaf is such that it allows for different Bv types as it
  // is only testing tree *topology*.
  BvNode<Aabb, TriangleSurfaceMesh<double>> aabb_leaf_of_one_element(
      Aabb(Vector3d::Zero(), Vector3d::Zero()), {1, {0}});
  EXPECT_TRUE(leaf_of_one_element.EqualLeaf(aabb_leaf_of_one_element));
}

// @tparam BvNodeType is either BvNode<TriangleSurfaceMesh<double>> or
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

// @tparam BvNodeType is either BvNode<TriangleSurfaceMesh<double>> or
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
// @tparam BvNodeType is either BvNode<TriangleSurfaceMesh<double>> or
//                    BvNode<VolumeMesh<double>>.
template <class BvNodeType>
int ComputeHeight(const BvNodeType& root) {
  if (root.is_leaf()) {
    return 0;
  }
  return 1 + std::max(ComputeHeight(root.left()),
                      ComputeHeight(root.right()));
}

// Test fixture class for testing Bvh. Uses a coarse sphere, i.e. an
// octahedron, as the underlying mesh. We expect the resultant BVH to be a
// perfect binary tree as tested in TestBuildBvTree.
template <typename BvType>
class BvhTest : public ::testing::Test {
 public:
  BvhTest()
      : ::testing::Test(),
        mesh_(MakeSphereSurfaceMesh<double>(Sphere(1.5), 3)),
        bvh_(Bvh<BvType, TriangleSurfaceMesh<double>>(mesh_)) {}

 protected:
  TriangleSurfaceMesh<double> mesh_;
  Bvh<BvType, TriangleSurfaceMesh<double>> bvh_;
};

using BvTypes = ::testing::Types<Aabb, Obb>;
TYPED_TEST_SUITE(BvhTest, BvTypes);

// Tests that the right element indices are used in the calculation. We can
// use a "toy" example because we're not testing the robustness of the BV
// calculations (those are tested in aabb/obb_test.cc). We're merely testing the
// propagation of the parameters.
TYPED_TEST(BvhTest, TestComputeBoundingVolume) {
  using BvType = TypeParam;
  // A mesh of two triangles that are well separated by X-Y plane.
  const TriangleSurfaceMesh<double> mesh(
      {{0, 1, 2}, {3, 4, 5}},
      {Vector3d(0, 0, 1), Vector3d(1, 0, 1), Vector3d(0, 1, 1),
       Vector3d(0, 0, -1), Vector3d(1, 0, -1), Vector3d(0, 1, -1)});

  using FaceCentroidPair = std::pair<int, Vector3d>;
  // The positions of centroids are not relevant to this test.
  std::vector<FaceCentroidPair> upper{{0, Vector3d()}};
  std::vector<FaceCentroidPair> lower{{1, Vector3d()}};

  const BvType bv =
      BvhTester::ComputeBoundingVolume<BvType, TriangleSurfaceMesh<double>>(
          mesh, upper.begin(), upper.end());
  EXPECT_TRUE(BvhTester::Contain(bv, mesh, upper.begin(), upper.end()));
  EXPECT_FALSE(BvhTester::Contain(bv, mesh, lower.begin(), lower.end()));
}

// Tests properties from building the bounding volume tree.
TYPED_TEST(BvhTest, TestBuildBvTree) {
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
  using BvType = TypeParam;
  EXPECT_EQ(CountAllNodes(this->bvh_.root_node()), 7);
  EXPECT_EQ(CountLeafNodes(this->bvh_.root_node()), 4);
  const int tree_height = ComputeHeight(this->bvh_.root_node());
  EXPECT_EQ(tree_height, 2);

  const int num_elements = this->mesh_.num_elements();
  std::set<int> element_indices;
  std::function<void(const BvNode<BvType, TriangleSurfaceMesh<double>>&, int)>
      check_node;
  check_node = [&check_node, &element_indices, num_elements, tree_height](
                   const BvNode<BvType, TriangleSurfaceMesh<double>>& node,
                   int depth) {
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
  check_node(this->bvh_.root_node(), 0);
  // Check that we found a leaf node for all elements.
  EXPECT_EQ(element_indices.size(), num_elements);
}

// Tests copy constructor.
TYPED_TEST(BvhTest, TestCopy) {
  // Copy constructor.
  using BvType = TypeParam;
  Bvh<BvType, TriangleSurfaceMesh<double>> bvh_copy(this->bvh_);

  // Confirm that it's a deep copy.
  std::function<void(const BvNode<BvType, TriangleSurfaceMesh<double>>&,
                     const BvNode<BvType, TriangleSurfaceMesh<double>>&)>
      check_copy;
  check_copy = [&check_copy](
                   const BvNode<BvType, TriangleSurfaceMesh<double>>& orig,
                   const BvNode<BvType, TriangleSurfaceMesh<double>>& copy) {
    EXPECT_NE(&orig, &copy);
    if (orig.is_leaf()) {
      ASSERT_TRUE(copy.is_leaf());
      EXPECT_TRUE(orig.EqualLeaf(copy));
    } else {
      check_copy(orig.left(), copy.left());
      check_copy(orig.right(), copy.right());
    }
  };
  check_copy(this->bvh_.root_node(), bvh_copy.root_node());
}

// Tests colliding while traversing through the bvh trees. We want to ensure
// that the case of no overlap is covered as well as the 4 cases of branch and
// leaf comparisons, i.e:
//  1. branch : branch
//  2. branch : leaf
//  3. leaf : branch
//  4. leaf : leaf
TYPED_TEST(BvhTest, TestCollide) {
  // The two trees are completely separate so no bounding volumes overlap and
  // all pairs should be culled. The resulting vector should thus be empty.
  using BvType = TypeParam;
  auto separate_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  RigidTransformd X_WV{Vector3d{4, 4, 4}};
  Bvh<BvType, TriangleSurfaceMesh<double>> separate(separate_mesh);
  std::vector<std::pair<int, int>> pairs =
      this->bvh_.GetCollisionCandidates(separate, X_WV);
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
  Bvh<BvType, TriangleSurfaceMesh<double>> tangent(tangent_mesh);
  // Reality check that the tree of `tangent` is a perfect binary tree. A
  // perfect binary tree with height H has number of leaves L satisfying
  // L = 2^H. We expect every leaf to have two triangles which leads to
  // 32/2 = 16 leaf nodes. That yields an expected height of 4. As an extra
  // check, we also confirm the number of nodes 2^(4+1)-1 = 31.
  ASSERT_EQ(ComputeHeight(tangent.root_node()), 4);
  ASSERT_EQ(CountLeafNodes(tangent.root_node()), 16);
  ASSERT_EQ(CountAllNodes(tangent.root_node()), 31);

  // The total number of collision candidates is an empirical observation only
  // and differs by BvType. This encodes the two observed values. The
  // significance here is that we get the same number in both cases. We
  // assume that equal number implies same contents.
  constexpr int kTotalCandidates = std::is_same_v<BvType, Obb> ? 100 : 32;
  pairs = this->bvh_.GetCollisionCandidates(tangent, X_WV);
  EXPECT_EQ(pairs.size(), kTotalCandidates);
  pairs = tangent.GetCollisionCandidates(this->bvh_, X_WV);
  EXPECT_EQ(pairs.size(), kTotalCandidates);
}

// Tests colliding while traversing through the bvh trees but with early exit.
// We want to ensure that the trees are not fully traversed. One way to test
// this is to count towards a limit as the condition for the exit. If we
// specify a limit that is less than the number of potentially colliding pairs,
// we can verify that the traversal has exited since our result can be no more
// than this limit.
TYPED_TEST(BvhTest, TestCollideEarlyExit) {
  int count{0};
  int limit{1};
  // This callback should only be run as many times as the specified limit
  // before the early exit kicks in.
  auto callback = [&count, &limit](int a, int b) -> BvttCallbackResult {
    ++count;
    return count < limit ? BvttCallbackResult::Continue
                         : BvttCallbackResult::Terminate;
  };
  // Since we're colliding bvh against itself there should be up to n^2
  // potentially colliding pairs, but we max out at our limit of 1.
  this->bvh_.Collide(this->bvh_, math::RigidTransformd::Identity(), callback);
  EXPECT_EQ(count, 1);

  count = 0;
  limit = 5;
  // Updating the limit to 5 should get further in the traversal with a result
  // of 5 pairs.
  this->bvh_.Collide(this->bvh_, math::RigidTransformd::Identity(), callback);
  EXPECT_EQ(count, 5);
}

// Confirms that we can collide bvh trees on different mesh types: surface vs.
// volume. We construct two meshes with known intersection and confirm that
// the BVH produces candidates which include the intersecting elements, and
// the total number of candidates is less than M * N.
TYPED_TEST(BvhTest, TestCollideSurfaceVolume) {
  using BvType = TypeParam;
  /* The meshes are defined as follows:

         E        A          Q
         │╲      ╱│         ╱│╲                  z
         │ ╲    ╱ │        ╱ │ ╲                 │
         │  ╲ D╱  │     P ╱  │  ╲ R              │
         │G  ╲╱  C│       ╲  │  ╱                │
         │   ╱╲   │        ╲ │ ╱                 o─────── x
         │  ╱  ╲  │         ╲│╱
         │ ╱    ╲ │          S
         │╱      ╲│
         F        B
         Vol. Mesh       Surf. Mesh

   - Volume mesh: two tetrahedra with a single shared vertex (D). Each tet has a
     triangular face (ABC and EFG, respectively) that lies parallel to the
     x = 0 plane.
   - Surface mesh: Two triangles with a shared edge (bd) lying on the y = 0
     plane.

   We'll pose the two meshes so that vertex P penetrates the tetrahedron face
   ABC. By this we know we'll have a single intersecting element pair (and their
   indices).

   The test will confirm that the culling works by looking at two things:

     1. The element pair including the intersecting tri-tet is passed to the
        callback.
     2. Culling takes place; there are 2 tets and 2 triangles leading to a total
        of four possible pairs. We'll confirm that the callback is invoked on
        *fewer* than four pairs. More particularly, we *expect* it to be invoked
        for two pairs, because the bvh for the volume mesh contains a *single*
        tet in each leaf, but for the surface mesh, both triangles will be
        contained in a single leaf. That means the bounding volumes will lead
        to exactly *two* invocations of the callback (tet, tri1) and
        (tet, tri2). */

  std::vector<Vector3d> vertices_S{{Vector3d(0, 0, 0),     // P
                                    Vector3d(1, 0, 1),     // Q
                                    Vector3d(2, 0, 0),     // R
                                    Vector3d(1, 0, -1)}};  // S
  std::vector<SurfaceTriangle> faces_S{{0, 3, 1}, {3, 2, 1}};
  const TriangleSurfaceMesh<double> mesh_S(std::move(faces_S),
                                           std::move(vertices_S));
  const Bvh<BvType, TriangleSurfaceMesh<double>> bvh_S(mesh_S);
  // Confirm the expected topology (one leaf with two tris).
  ASSERT_EQ(CountLeafNodes(bvh_S.root_node()), 1);

  std::vector<Vector3d> vertices_V{
      {Vector3d(1, 0, 1),     // A
       Vector3d(1, 0, -1),    // B
       Vector3d(1, 1, 0),     // C
       Vector3d(0, 0, 0),     // D
       Vector3d(-1, 0, 1),    // E
       Vector3d(-1, 0, -1),   // F
       Vector3d(-1, 1, 0)}};  // G
  std::vector<VolumeElement> tets_V{{0, 2, 1, 3}, {4, 5, 6, 3}};
  const VolumeMesh<double> mesh_V(std::move(tets_V), std::move(vertices_V));
  const Bvh<BvType, VolumeMesh<double>> bvh_V(mesh_V);
  // Confirm the expected topology (two leaves, one per tet).
  ASSERT_EQ(CountLeafNodes(bvh_V.root_node()), 2);

  const RigidTransformd X_VS(Vector3d(0.95, 0.5, 0.01));

  /* Use the callback to record if the intersecting element pair is included in
   the candidates and the total number of candidates we get. */
  bool intersecting_included = false;
  int total_count = 0;
  auto callback = [&intersecting_included, &total_count](int v, int s) {
    ++total_count;
    if (v == 0 && s == 0) intersecting_included = true;
    return BvttCallbackResult::Continue;
  };

  bvh_V.Collide(bvh_S, X_VS, callback);

  EXPECT_TRUE(intersecting_included);
  EXPECT_EQ(total_count, 2);
  EXPECT_LT(total_count, mesh_S.num_elements() * mesh_V.num_elements());
}

// Tests colliding the bvh tree with a primitive object. We use a plane as a
// representative primitive.
TYPED_TEST(BvhTest, TestCollidePrimitive) {
  using BvType = TypeParam;
  // TODO(SeanCurtis-TRI): When Aabb supports plane and half space intersection
  //  expand this test to include Aabb.
  if constexpr (std::is_same_v<BvType, Obb>) {
    // Use Y-Z plane of frame P, i.e., the plane is perpendicular to Px.
    // Using such a trivial primitive is fine because we're not testing the
    // robustness of primitive-obb algorithms, but the Bvh algorithm that
    // iterates through them. We rely on the obb-primitive tests to handle the
    // robustness.
    Plane<double> plane_P{Vector3d::UnitX(), Vector3d::Zero()};

    // All collision candidates.
    {
      std::vector<int> candidates;
      // We pose the plane to pass through a plane of symmetry of the
      // octahedron, so we should get all 8 triangles.
      this->bvh_.Collide(
          plane_P, RigidTransformd::Identity(),
          [&candidates](int f) -> BvttCallbackResult {
            candidates.push_back(f);
            return BvttCallbackResult::Continue;
          });
      int num_candidates = candidates.size();
      EXPECT_EQ(num_candidates, 8);
    }

    // First collision candidate then terminate early.
    {
      std::vector<int> candidates;
      this->bvh_.Collide(
          plane_P, RigidTransformd::Identity(),
          [&candidates](int f) -> BvttCallbackResult {
            candidates.push_back(f);
            return BvttCallbackResult::Terminate;
          });
      int num_candidates = candidates.size();
      EXPECT_EQ(num_candidates, 1);
    }

    // No collision.
    {
      std::vector<int> candidates;
      // We pose the plane far from the mesh.
      this->bvh_.Collide(
          plane_P, RigidTransformd(Vector3d(10, 0, 0)),
          [&candidates](int f) -> BvttCallbackResult {
            candidates.push_back(f);
            return BvttCallbackResult::Continue;
          });
      int num_candidates = candidates.size();
      EXPECT_EQ(num_candidates, 0);
    }
  }
}

// Tests computing the centroid of an element.
TYPED_TEST(BvhTest, TestComputeCentroid) {
  using BvType = TypeParam;
  // Set resolution at double so that we get the coarsest mesh of 8 elements.
  auto surface_mesh =
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6);
  Vector3d centroid =
      BvhTester::ComputeCentroid<BvType, TriangleSurfaceMesh<double>>(
          surface_mesh, 0);
  // The first face of our octahedron is a triangle with vertices at 1, 2, and
  // 3 along each respective axis, so its centroid should average out to 1/3,
  // 2/3, and 3/3.
  EXPECT_TRUE(CompareMatrices(centroid, Vector3d(1. / 3., 2. / 3., 1.)));

  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1., 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  centroid = BvhTester::ComputeCentroid<BvType, VolumeMesh<double>>(
      volume_mesh, 0 /* tet_index */);
  // The first face of our octahedron is a tet with vertices at 1, 2, and 3
  // along each respective axis and the origin 0, so its centroid should
  // average out to 1/4, 2/4, and 3/4.
  EXPECT_TRUE(CompareMatrices(centroid, Vector3d(0.25, 0.5, 0.75)));
}

TYPED_TEST(BvhTest, TestEqual) {
  using BvType = TypeParam;
  // Tests that two BVHs with the same tree structure but different
  // bounding volumes are not equal. Each tree has only one node.
  {
    const VolumeMesh<double> smaller_tetrahedron(
        std::vector<VolumeElement>{VolumeElement(0, 1, 2, 3)},
        std::vector<Vector3<double>>{Vector3d::Zero(), Vector3d::UnitX(),
                                     Vector3d::UnitY(), Vector3d::UnitZ()});
    const Bvh<BvType, VolumeMesh<double>> smaller(smaller_tetrahedron);
    const VolumeMesh<double> bigger_tetrahedron(
        std::vector<VolumeElement>{VolumeElement(0, 1, 2, 3)},
        std::vector<Vector3<double>>{Vector3d::Zero(), 2. * Vector3d::UnitX(),
                                     2. * Vector3d::UnitY(),
                                     2. * Vector3d::UnitZ()});
    const Bvh<BvType, VolumeMesh<double>> bigger(bigger_tetrahedron);
    // Verify that each tree has the same structure. Each has only one node.
    ASSERT_TRUE(smaller.root_node().is_leaf());
    ASSERT_TRUE(bigger.root_node().is_leaf());
    // Verify that the two trees have different bounding volumes. To avoid
    // numerical methods involved in computing the BVs, we're not going to
    // test for a *specific* relationship, but merely assert inequality.
    ASSERT_FALSE(smaller.root_node().bv().Equal(bigger.root_node().bv()));

    EXPECT_FALSE(smaller.Equal(bigger));
    EXPECT_FALSE(bigger.Equal(smaller));
  }

  // Tests that two BVHs are not equal due to different tree structures even
  // though every node has the same bounding volume.
  {
    const std::vector<Vector3<double>> vertices{
        Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
        Vector3d::UnitZ()};
    const VolumeElement element(0, 1, 2, 3);
    const VolumeMesh<double> one_tetrahedron(
        std::vector<VolumeElement>{element},
        std::vector<Vector3d>(vertices));
    const Bvh<BvType, VolumeMesh<double>> one_node(one_tetrahedron);
    const VolumeMesh<double> duplicated_tetrahedra(
        // Both elements are the same tetrahedron, so every Bvh node will have
        // the same bounding volume.
        std::vector<VolumeElement>{element, element},
        std::vector<Vector3d>(vertices));
    const Bvh<BvType, VolumeMesh<double>> three_nodes(duplicated_tetrahedra);
    // Verify that the two trees have different structures.
    ASSERT_TRUE(one_node.root_node().is_leaf());
    ASSERT_FALSE(three_nodes.root_node().is_leaf());
    ASSERT_TRUE(three_nodes.root_node().left().is_leaf());
    ASSERT_TRUE(three_nodes.root_node().right().is_leaf());
    // Verify that every node of the two trees has the same bounding volume.
    const BvType expect_bv = one_node.root_node().bv();
    ASSERT_TRUE(three_nodes.root_node().bv().Equal(expect_bv));
    ASSERT_TRUE(three_nodes.root_node().left().bv().Equal(expect_bv));
    ASSERT_TRUE(three_nodes.root_node().right().bv().Equal(expect_bv));

    EXPECT_FALSE(one_node.Equal(three_nodes));
    EXPECT_FALSE(three_nodes.Equal(one_node));
  }

  // Tests equality of BVH's that require multiple recursive calls in
  // traversing the trees. The mesh has enough triangles for the tree to have
  // enough depth.
  const TriangleSurfaceMesh<double> mesh_ellipsoid =
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6.);
  ASSERT_EQ(8, mesh_ellipsoid.num_triangles());
  const Bvh<BvType, TriangleSurfaceMesh<double>> bvh_ellipsoid(mesh_ellipsoid);
  const Bvh<BvType, TriangleSurfaceMesh<double>> bvh_ellipsoid_too(
      mesh_ellipsoid);
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
TYPED_TEST(BvhTest, BvhFromAutodiffmesh) {
  using BvType = TypeParam;
  TriangleSurfaceMesh<AutoDiffXd> mesh_ad =
      MakeSphereSurfaceMesh<AutoDiffXd>(Sphere(1.5), 3);
  Bvh<BvType, TriangleSurfaceMesh<AutoDiffXd>> bvh_ad(mesh_ad);
  TriangleSurfaceMesh<double> mesh_d =
      MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  Bvh<BvType, TriangleSurfaceMesh<double>> bvh_d(mesh_d);
  EXPECT_TRUE(bvh_ad.Equal(bvh_d));
}

// This confirms that we can execute the Collide() method between Bvhs built on
// different bounding volume types. This is largely a smoke test. It confirms
// that the invocations build and execute without throwing. We check for
// an *indicator* of correctness: overlapping geometries produce non-zero
// results, separated geometries produce no results.
//
// As we implement further bounding volume types, this test should be extended
// to exercise those additional permutations.
GTEST_TEST(BoundingVolumeHierarchyTest, CollideDifferentBvTypes) {
  TriangleSurfaceMesh<double> mesh =
      MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  Bvh<Aabb, TriangleSurfaceMesh<double>> aabb_bvh(mesh);
  Bvh<Obb, TriangleSurfaceMesh<double>> obb_bvh(mesh);

  std::vector<std::pair<int, int>> results;
  auto callback = [&results](int a, int b) {
    results.emplace_back(a, b);
    return BvttCallbackResult::Continue;
  };

  aabb_bvh.Collide(obb_bvh, RigidTransformd{}, callback);
  ASSERT_GT(results.size(), 0);
  results.clear();

  aabb_bvh.Collide(obb_bvh, RigidTransformd{Vector3d{100, 0, 0}}, callback);
  EXPECT_EQ(results.size(), 0);
  results.clear();

  obb_bvh.Collide(aabb_bvh, RigidTransformd{}, callback);
  ASSERT_GT(results.size(), 0);
  results.clear();

  obb_bvh.Collide(aabb_bvh, RigidTransformd{Vector3d{100, 0, 0}}, callback);
  EXPECT_EQ(results.size(), 0);
}

// This confirms that we can execute the GetCollisionCandidates() method between
// Bvhs built on different bounding volume types. This is largely a smoke test.
// It confirms that the invocations build and execute without throwing. We check
// for an *indicator* of correctness: overlapping geometries produce non-zero
// results, separated geometries produce no results.
//
// As we implement further bounding volume types, this test should be extended
// to exercise those additional permutations.
GTEST_TEST(BoundingVolumeHierarchyTest, CandidatesDifferentBvTypes) {
  TriangleSurfaceMesh<double> mesh =
      MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  Bvh<Aabb, TriangleSurfaceMesh<double>> aabb_bvh(mesh);
  Bvh<Obb, TriangleSurfaceMesh<double>> obb_bvh(mesh);

  EXPECT_GT(aabb_bvh.GetCollisionCandidates(obb_bvh, RigidTransformd{}).size(),
            0);
  EXPECT_EQ(
      aabb_bvh
          .GetCollisionCandidates(obb_bvh, RigidTransformd{Vector3d{100, 0, 0}})
          .size(),
      0);

  EXPECT_GT(obb_bvh.GetCollisionCandidates(aabb_bvh, RigidTransformd{}).size(),
            0);
  EXPECT_EQ(obb_bvh
                .GetCollisionCandidates(aabb_bvh,
                                        RigidTransformd{Vector3d{100, 0, 0}})
                .size(),
            0);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
