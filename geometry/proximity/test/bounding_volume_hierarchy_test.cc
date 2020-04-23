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
using math::RotationMatrixd;

// Friend class for accessing AAbb's protected/private functionality.
class AabbTester : public ::testing::Test {
 public:
  static constexpr double kTolerance = Aabb::kTolerance;
};

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
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), Vector3d(1.5, 1.5, 1.5),
                              AabbTester::kTolerance));

  // Test with a volume mesh. As above, the centroids are still irrelevant. The
  // bounding box should encompass the whole ellipsoid with a center of 0 and
  // half width of 1, 2, 3.
  std::vector<std::pair<VolumeElementIndex, Vector3d>> tet_centroids;
  auto volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1., 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  for (VolumeElementIndex i(0); i < volume_mesh.num_elements(); ++i) {
    tet_centroids.emplace_back(i, Vector3d(0.5, 0.5, 0.5));
  }
  aabb = BVHTester::ComputeBoundingVolume<VolumeMesh<double>>(
      volume_mesh, tet_centroids.begin(), tet_centroids.end());
  EXPECT_TRUE(CompareMatrices(aabb.center(), Vector3d(0., 0., 0.)));
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), Vector3d(1., 2., 3.),
                              AabbTester::kTolerance));
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
                     const BvNode<SurfaceMesh<double>>&)>
      check_copy;
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
  BoundingVolumeHierarchy<SurfaceMesh<double>> separate(separate_mesh);
  std::vector<std::pair<SurfaceFaceIndex, SurfaceFaceIndex>> pairs =
      bvh_.GetCollisionCandidates(separate, X_WV);
  EXPECT_EQ(pairs.size(), 0);

  // Create a higher resolution mesh so we have a different number of elements
  // across the bvh trees, i.e. 8 in our coarse octahedron and 32 here. We
  // place the meshes such that they are touching at one corner, so the
  // traversal will reach end leaf-leaf cases (4.) since there are potentially
  // colliding pairs. Since the trees have different depths, the traversal
  // will cover branch-leaf cases (2.) on its way. Swapping the order then
  // catches the opposing leaf-branch cases (3.). Since the trees have multiple
  // depths then at higher levels, for example at the root, the branch-branch
  // cases (1.) will be covered.
  auto tangent_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 2);
  X_WV = RigidTransformd{Vector3d{3, 0, 0}};
  BoundingVolumeHierarchy<SurfaceMesh<double>> tangent(tangent_mesh);
  pairs = bvh_.GetCollisionCandidates(tangent, X_WV);
  EXPECT_EQ(pairs.size(), 16);
  pairs = tangent.GetCollisionCandidates(bvh_, X_WV);
  EXPECT_EQ(pairs.size(), 16);
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
  BoundingVolumeHierarchy<VolumeMesh<double>> tet_bvh(volume_mesh);
  auto surface_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  RigidTransformd X_WV{Vector3d{3, 0, 0}};
  BoundingVolumeHierarchy<SurfaceMesh<double>> tri_bvh(surface_mesh);

  auto pairs = tet_bvh.GetCollisionCandidates(tri_bvh, X_WV);
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

// Tests calculating the bounding box volume. Due to boundary padding, the
// volume is increased from 8abc to 8((a + ε)*(b + ε)*(c+ε)), i.e.:
// 8[abc + (ab + bc + ac)ε + (a + b + c)ε² + ε³].
TEST_F(AabbTester, TestVolume) {
  Aabb aabb = Aabb(Vector3d(-1, 2, 1), Vector3d(2, 0.5, 2.7));
  // In this case the dominating error term is 8(ab + bc + ac)ε, which caps
  // out under kTolerance * 70.
  const double volume = aabb.CalcVolume();
  EXPECT_NEAR(volume, 21.6, kTolerance * 70);
  EXPECT_GT(volume, 21.6);
  Aabb zero_aabb = Aabb(Vector3d(3, -4, 1.3), Vector3d(0, 0, 0));
  // Since a, b and c are 0, only the ε³ term is left and kTolerance³ is
  // within kTolerance.
  const double zero_volume = zero_aabb.CalcVolume();
  EXPECT_NEAR(zero_volume, 0, kTolerance);
  EXPECT_GT(zero_volume, 0);
}

// Tests calculating the bounding points.
TEST_F(AabbTester, TestBounds) {
  Aabb aabb = Aabb(Vector3d(-1, 2, 1), Vector3d(2, 0.5, 2.7));
  EXPECT_TRUE(
      CompareMatrices(aabb.upper(), Vector3d(1, 2.5, 3.7),
                      kTolerance + std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(
      CompareMatrices(aabb.lower(), Vector3d(-3, 1.5, -1.7),
                      kTolerance + std::numeric_limits<double>::epsilon()));
}

// Tests padding the boundary of the bounding box volume.
TEST_F(AabbTester, TestPadBoundary) {
  Aabb aabb = Aabb(Vector3d(-1, 0.5, 1), Vector3d(1.2, 2.5, 0.3));
  Vector3d padded = Vector3d(1.2, 2.5, 0.3).array() + kTolerance;
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), padded));

  // Large boxes should have a bigger padding based on either the maximum
  // half width or position in the frame.
  const double padding = 300 * std::numeric_limits<double>::epsilon();
  ASSERT_GT(padding, kTolerance);
  // Max is set from half_width.z.
  aabb = Aabb(Vector3d(-1, 1.5, 1), Vector3d(120, 250, 300));
  padded = Vector3d(120, 250, 300).array() + padding;
  // Expect the two Vector3d to be exactly equal down to the last bit.
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), padded));
  // Max is set from |center.x|.
  aabb = Aabb(Vector3d(-300, 50, 100), Vector3d(1, 2, 0.5));
  padded = Vector3d(1, 2, 0.5).array() + padding;
  // Expect the two Vector3d to be exactly equal down to the last bit.
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), padded));
}

// We want to compute X_AB such that B is posed relative to A as documented in
// TestObbOverlap. We can do so by generating the rotation component, R_AB, such
// that Bq has a minimum value along the chosen axis, and we can solve for
// the translation component, p_AoBo = p_AoAf + p_AfBq + p_BqBo_A.
auto calc_corner_transform = [](const Aabb& a, const Aabb& b, const int axis,
                                const bool expect_overlap) -> RigidTransformd {
  const int axis1 = (axis + 1) % 3;
  const int axis2 = (axis + 2) % 3;
  // Construct the rotation matrix, R_AB, that has meaningful (non-zero)
  // values everywhere for the remaining 2 axes and no symmetry.
  RotationMatrixd R_AB =
      RotationMatrixd(AngleAxisd(M_PI / 5, Vector3d::Unit(axis1))) *
      RotationMatrixd(AngleAxisd(-M_PI / 5, Vector3d::Unit(axis2)));
  // We define p_BoBq in Frame A by taking the minimum corner and applying
  // the constructed rotation.
  Vector3d p_BoBq_A = R_AB * (b.center() - b.half_width());
  // Reality check that the center (p_BoBc_A) and the maximum corner
  // (p_BoBqprime_A) are strictly increasing along the given axis.
  Vector3d p_BoBc_A = R_AB * b.center();
  Vector3d p_BoBqprime_A = R_AB * (b.center() + b.half_width());
  DRAKE_DEMAND(p_BoBc_A[axis] > p_BoBq_A[axis]);
  DRAKE_DEMAND(p_BoBqprime_A[axis] > p_BoBc_A[axis]);
  // We construct Bq to be a small relative offset either side of Af along the
  // given axis, depending on whether we expect the boxes to overlap.
  Vector3d p_AfBq{0, 0, 0};
  p_AfBq[axis] = expect_overlap ? -0.01 : 0.01;
  // We construct Af by taking the maximum corner and offsetting it along the
  // remaining 2 axes, e.g. by a quarter across. This ensures we thoroughly
  // exercise all bits instead of simply using any midpoints or corners.
  // z
  // ^
  // |  -------------
  // |  |     |  o  |
  // |  |------------
  // |  |     |     |
  // |  -------------
  // -----------------> y
  Vector3d p_AoAf = a.half_width();
  p_AoAf[axis1] /= 2;
  p_AoAf[axis2] /= 2;
  p_AoAf += a.center();
  // We can rewrite +p_BqBo as -p_BoBq, thus solving for p_AoBo = p_AoAf +
  // p_AfBq - p_BoBq_A.
  Vector3d p_AoBo = p_AoAf + p_AfBq - p_BoBq_A;
  // Finally we combine the components to form the transform X_AB.
  return RigidTransformd(R_AB, p_AoBo);
};

// We want to compute X_AB such that B is posed relative to A as documented
// in TestObbOverlap. We can do so by generating the rotation component, R_AB,
// such that Bq lies on the minimum edge along the chosen axis, and we can solve
// for the translation component, p_AoBo = p_AoAf + p_AfBq + p_BqBo_A.
auto calc_edge_transform = [](const Aabb& a, const Aabb& b, const int a_axis,
                              const int b_axis,
                              const bool expect_overlap) -> RigidTransformd {
  const int a_axis1 = (a_axis + 1) % 3;
  const int a_axis2 = (a_axis + 2) % 3;
  const int b_axis1 = (b_axis + 1) % 3;
  const int b_axis2 = (b_axis + 2) % 3;
  // Construct a rotation matrix that has meaningful (non-zero) values
  // everywhere for the remaining 2 axes and no symmetry. Depending on the
  // combination of axes, we need to rotate around different axes to ensure
  // the edge remains as the minimum.
  RotationMatrixd R_AB;
  const double theta = M_PI / 5;
  // For cases Ax × Bx, Ay × By, and Az × Bz.
  if (a_axis == b_axis) {
    R_AB = RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1))) *
           RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis2)));
    // For cases Ax × By, Ay × Bz, and Az × Bx.
  } else if (a_axis1 == b_axis) {
    R_AB = RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1))) *
           RotationMatrixd(AngleAxisd(-theta, Vector3d::Unit(b_axis2)));
    // For cases Ax × Bz, Ay × Bx, and Az × By.
  } else {
    R_AB = RotationMatrixd(AngleAxisd(-theta, Vector3d::Unit(b_axis2))) *
           RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1)));
  }
  // We define p_BoBq in Frame B taking a point on the minimum edge aligned
  // with the given axis, offset it to be without symmetry, then convert it
  // to Frame A by applying the rotation.
  Vector3d p_BoBQ_B = b.center() - b.half_width();
  p_BoBQ_B[b_axis] += b.half_width()[b_axis] / 2;
  Vector3d p_BoBq_A = R_AB * p_BoBQ_B;
  // Reality check that the center (p_BoBc_A) and the point on the opposite
  // edge (p_BoBqprime_A) are strictly increasing along the remaining 2 axes.
  Vector3d p_BoBc_A = R_AB * b.center();
  Vector3d p_BoBqprime_B = b.center() + b.half_width();
  p_BoBqprime_B[b_axis] -= b.half_width()[b_axis] / 2;
  Vector3d p_BoBqprime_A = R_AB * p_BoBqprime_B;
  DRAKE_DEMAND(p_BoBc_A[a_axis1] > p_BoBq_A[a_axis1]);
  DRAKE_DEMAND(p_BoBqprime_A[a_axis1] > p_BoBc_A[a_axis1]);
  DRAKE_DEMAND(p_BoBc_A[a_axis2] > p_BoBq_A[a_axis2]);
  DRAKE_DEMAND(p_BoBqprime_A[a_axis2] > p_BoBc_A[a_axis2]);
  // We construct Bq to be a small relative offset either side of Af along the
  // given axis, depending on whether we expect the boxes to overlap.
  Vector3d p_AfBq{0, 0, 0};
  const double offset = expect_overlap ? -0.01 : 0.01;
  p_AfBq[a_axis1] = offset;
  p_AfBq[a_axis2] = offset;
  // We construct Af by taking the maximum corner and offsetting it along the
  // given edge to thoroughly exercise all bits.
  Vector3d p_AoAf = a.center() + a.half_width();
  p_AoAf[a_axis] -= a.half_width()[a_axis] / 2;
  // We can rewrite +p_BqBo as -p_BoBq, thus solving for p_AoBo = p_AoAf +
  // p_AfBq - p_BoBq_A.
  Vector3d p_AoBo = p_AoAf + p_AfBq - p_BoBq_A;
  // Finally we combine the components to form the transform X_AB.
  return RigidTransformd(R_AB, p_AoBo);
};

// Tests whether OBBs overlap. There are 15 cases to test, each covering a
// separating axis between the two bounding boxes. The first 3 cases use the
// axes of Frame A, the next 3 cases use the axes of Frame B, and the remaining
// 9 cases use the axes defined by the cross product of axes from Frame A and
// Frame B. We also test that it is robust for the case of parallel boxes.
GTEST_TEST(AABBTest, TestObbOverlap) {
  // One box is fully contained in the other and they are parallel.
  Aabb a = Aabb(Vector3d(1, 2, 3), Vector3d(1, 2, 1));
  Aabb b = Aabb(Vector3d(1, 2, 3), Vector3d(0.5, 1, 0.5));
  RigidTransformd X_AB = RigidTransformd::Identity();
  EXPECT_TRUE(Aabb::HasOverlap(a, b, X_AB));

  // To cover the cases of the axes of Frame A, we need to pose box B along
  // each axis. For example, in the case of the x-axis, in a 2D view they would
  // look like:
  // y
  // ^
  // |  -----------------       *
  // |  |               |     *   *
  // |  |      Ao       Af Bq   Bo  *
  // |  |               |     *       *
  // |  |               |       *   *
  // |  -----------------         *
  // -----------------------------------> x
  //
  // For this test, we define Point Bq as the minimum corner of the box B (i.e.,
  // center - half width). We want to pose box B so Bq is the uniquely closest
  // point to box A at a Point Af in the interior of its +x face. The rest of
  // the box extends farther along the +x axis (as suggested in the above
  // illustration). Point Bq will be a small epsilon away from the nearby face
  // either outside (if expect_overlap is false) or inside (if true).
  a = Aabb(Vector3d(1, 2, 3), Vector3d(2, 4, 3));
  b = Aabb(Vector3d(2, 0.5, 4), Vector3d(3.5, 2, 1.5));
  for (int axis = 0; axis < 3; ++axis) {
    X_AB = calc_corner_transform(a, b, axis, false /* expect_overlap */);
    EXPECT_FALSE(Aabb::HasOverlap(a, b, X_AB));
    X_AB = calc_corner_transform(a, b, axis, true /* expect_overlap */);
    EXPECT_TRUE(Aabb::HasOverlap(a, b, X_AB));
  }

  // To cover the local axes out of B, we can use the same method by swapping
  // the order of the boxes and then using the inverse of the transform.
  for (int axis = 0; axis < 3; ++axis) {
    X_AB =
        calc_corner_transform(b, a, axis, false /* expect_overlap */).inverse();
    EXPECT_FALSE(Aabb::HasOverlap(a, b, X_AB));
    X_AB =
        calc_corner_transform(b, a, axis, true /* expect_overlap */).inverse();
    EXPECT_TRUE(Aabb::HasOverlap(a, b, X_AB));
  }

  // To cover the remaining 9 cases, we need to pose an edge from box B along
  // an edge from box A. The axes that the edges are aligned with form the
  // two inputs into the cross product for the separating axis. For example,
  // in the following illustration, Af lies on the edge aligned with A's y-axis.
  // Assuming that Bq lies on an edge aligned with B's x-axis, this would form
  // the case testing the separating axis Ay × Bx.
  //                       _________
  //   +z                 /________/\              .
  //    ^                 \        \ \             .
  //    |   ______________ Bq       \ \            .
  //    |  |\             Af \  Bo   \ \           .
  //    |  | \ _____________\ \       \ \          .
  // +y |  | |      Ao      |  \_______\/          .
  //  \ |  \ |              |                      .
  //   \|   \|______________|                      .
  //    -----------------------------------> +x
  //
  // For this test, we define point Bq on the minimum edge of the box in its
  // own frame (i.e., center - half width + an offset along the edge). We want
  // to pose box B so Bq is the uniquely closest point to A at a Point Af on the
  // edge between the +x and +z face of box A. The rest of the box extends
  // farther along the +x and +z axis (as suggested in the above illustration).
  // Point Bq will be a small epsilon away from the nearby edge either outside
  // (if expect_overlap is false) or inside (if true).
  for (int a_axis = 0; a_axis < 3; ++a_axis) {
    for (int b_axis = 0; b_axis < 3; ++b_axis) {
      X_AB =
          calc_edge_transform(a, b, a_axis, b_axis, false /* expect_overlap */);
      // Separate along a's y-axis and b's x-axis.
      EXPECT_FALSE(Aabb::HasOverlap(a, b, X_AB));
      X_AB =
          calc_edge_transform(a, b, a_axis, b_axis, true /* expect_overlap */);
      // Separate along a's y-axis and b's x-axis.
      EXPECT_TRUE(Aabb::HasOverlap(a, b, X_AB));
    }
  }
}

// Tests Aaab-plane intersection. We have four frames:
//
//   B: the canonical frame the box is defined in (centered on Bo and aligned
//      with B's axes).
//   H: the frame in which the box B is positioned (i.e., R_HB = I, but
//      Ho != Bo.)
//   P: the frame the plane is defined in.
//   Q: the frame the query is performed in.
//
// For simplicity, we'll define the plane with a normal in the Pz direction
// passing through Po. We'll pose the box relative to the plane (so we can
// easily reason about whether it penetrates or not). But then express the plane
// in the query frame Q (and the pose of B in Q).
GTEST_TEST(AabbTest, PlaneOverlap) {
  // The aabb is *not* defined at the origin of the hierarchy frame.
  const Vector3d p_HoBo_H = Vector3d{0.5, 0.25, -0.75};
  const Vector3d half_width{1, 2, 3};
  Aabb aabb_H{p_HoBo_H, half_width};

  // Use brute force to find the position of the "lowest" corner of the box
  // measured from Ho and expressed in frame P. "Lowest" means the corner with
  // the smallest z-component. Note: the "z-component" trick only works because
  // we expect the plane to be Pz = 0.
  auto lowest_corner = [&half_width, &p_HoBo_H](const RotationMatrixd& R_PH) {
    Vector3d p_HoCmin_P =
        Vector3d::Constant(std::numeric_limits<double>::infinity());
    for (const double x_sign : {-1.0, 1.0}) {
      for (const double y_sign : {-1.0, 1.0}) {
        for (const double z_sign : {-1.0, 1.0}) {
          const Vector3d signs{x_sign, y_sign, z_sign};
          const Vector3d p_BoC_H = half_width.cwiseProduct(signs);
          const Vector3d p_HoC_P = R_PH * (p_HoBo_H + p_BoC_H);
          if (p_HoC_P(2) < p_HoCmin_P(2)) {
            p_HoCmin_P = p_HoC_P;
          }
        }
      }
    }
    return p_HoCmin_P;
  };

  // Test epsilon is the product of three factors:
  //  - machine epsilon
  //  - Two orders of magnitude attributed to the various transformations.
  //  - A scale factor that is the maximum of (box size, p_HoBo, p_PoHo)
  const double kEps = 300 * std::numeric_limits<double>::epsilon();
  // An arbitrary collection of orientations for the box's hierarchy frame H
  // in the plane frame P.
  std::vector<AngleAxisd> R_PHs{
      AngleAxisd{0, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitY()},
      AngleAxisd{M_PI / 2, Vector3d::UnitZ()},
      AngleAxisd{M_PI / 4, Vector3d::UnitX()},
      AngleAxisd{M_PI / 4, Vector3d::UnitY()},
      AngleAxisd{M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      AngleAxisd{7 * M_PI / 6, Vector3d{-1, 2, -3}.normalized()},
      AngleAxisd{12 * M_PI / 7, Vector3d{1, -2, 3}.normalized()}
  };
  // An arbitrary collection of poses of the plane in the query frame Q.
  std::vector<RigidTransformd> X_QPs{
      RigidTransformd{},  // Identity matrix.
      RigidTransformd{
          RotationMatrixd{AngleAxisd{M_PI / 4, Vector3d{1, 2, 3}.normalized()}},
          Vector3d{1, 2, 3}},
      RigidTransformd{RotationMatrixd{AngleAxisd{
                          12 * M_PI / 7, Vector3d{-1, -1, 3}.normalized()}},
                      Vector3d{-3, -1, 2}}
  };
  for (const auto& angle_axis_PH : R_PHs) {
    const RotationMatrixd R_PH{angle_axis_PH};
    const Vector3d p_HoCmin_P = lowest_corner(R_PH);
    for (const auto& X_QP : X_QPs) {
      // Define the plane in the query frame Q.
      const Vector3d& Pz_Q = X_QP.rotation().col(2);
      Plane<double> plane_Q{Pz_Q, X_QP.translation()};

      // We position Ho such that Cmin lies on the z = 0 plane in Frame P. Given
      // we know p_HoCmin_P, we know its current z-value. To put it at zero, we
      // must displace it in the negative of that z value. The x- and y-values
      // don't matter, so we pick values we know not to be zero.
      {
        // Place the minimum corner just "above" the plane.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmin_P(2) + kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_FALSE(Aabb::HasOverlap(aabb_H, plane_Q, X_QP * X_PH));
      }
      {
        // Place the minimum corner just "below" the plane.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmin_P(2) - kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_TRUE(Aabb::HasOverlap(aabb_H, plane_Q, X_QP * X_PH));
      }

      // We repeat the same task but with Cmax. Cmax is the reflection of Cmin
      // over Bo (the origin of the box). We'll express all vectors in the P
      // frame so we can place that corner just above and below the Pz = 0
      // plane using the same trick as documented above.
      const Vector3d p_HoBo_P = R_PH * p_HoBo_H;
      const Vector3d p_HoCmax_P = p_HoCmin_P + 2 * (p_HoBo_P - p_HoCmin_P);
      {
        // Put the maximum corner *on* the z = 0 plane in Frame P. The bulk of
        // the box now extends *below* the plane; so bump it up epsilon to
        // guarantee intersection.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmax_P(2) + kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_TRUE(Aabb::HasOverlap(aabb_H, plane_Q, X_QP * X_PH));
      }
      {
        // Put the maximum corner *on* the z = 0 plane in Frame P. The bulk of
        // the box now extends *below* the plane; so bump it down epsilon to
        // guarantee _no_ intersection.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmax_P(2) - kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_FALSE(Aabb::HasOverlap(aabb_H, plane_Q, X_QP * X_PH));
      }
    }
  }
}


// Tests the determination of an Aabb intersects a half space.
GTEST_TEST(AabbTest, HalfSpaceOverlap) {
  // We'll rely on the pose to reposition the box.
  const Vector3d p_HoBo_H = Vector3d{0.25, -0.5, 0.75};
  const Vector3d half_width{1, 2, 3};
  Aabb aabb_H{p_HoBo_H, half_width};

  // Find the "lowest" corner of the aabb relative to the half space. That means
  // the corner that has the smallest "z" value when expressed in Frame C.
  // We return the corner measured in the H frame and expressed in the C frame.
  // We use a brute force method to distinguish from the "cleverness" in the
  // Aabb algorithm.
  auto lowest_corner = [&half_width, &p_HoBo_H](const RotationMatrixd& R_CH) {
    Vector3d p_HoVmin_C =
        Vector3d::Constant(std::numeric_limits<double>::infinity());
    for (const double x_sign : {-1.0, 1.0}) {
      for (const double y_sign : {-1.0, 1.0}) {
        for (const double z_sign : {-1.0, 1.0}) {
          const Vector3d signs{x_sign, y_sign, z_sign};
          const Vector3d p_BoV_H = half_width.cwiseProduct(signs);
          const Vector3d p_HoV_H = p_HoBo_H + p_BoV_H;
          const Vector3d p_HoV_C = R_CH * p_HoV_H;
          if (p_HoV_C(2) < p_HoVmin_C(2)) {
            p_HoVmin_C = p_HoV_C;
          }
        }
      }
    }
    return p_HoVmin_C;
  };

  const double kEps = 100 * std::numeric_limits<double>::epsilon();
  // An arbitrary collection of orientations.
  std::vector<AngleAxisd> R_CHs{
      AngleAxisd{0, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitY()},
      AngleAxisd{M_PI / 2, Vector3d::UnitZ()},
      AngleAxisd{M_PI / 4, Vector3d::UnitX()},
      AngleAxisd{M_PI / 4, Vector3d::UnitY()},
      AngleAxisd{M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      AngleAxisd{7 * M_PI / 6, Vector3d{-1, 2, -3}.normalized()},
      AngleAxisd{12 * M_PI / 7, Vector3d{1, -2, 3}.normalized()}
  };
  const HalfSpace hs_C;

  for (const auto& angle_axis_CH : R_CHs) {
    const RotationMatrixd R_CH{angle_axis_CH};
    const Vector3d p_HoVmin_C = lowest_corner(R_CH);
    // We position Ho such that Vmin lies on the z = 0 plane in Frame C. Given
    // we know p_HoVmin_C, we know its current z-value. To put it at zero, we
    // must displace it in the negative of that z value. The x- and y-values
    // don't matter, so we pick values we know not to be zero.
    {
      // Place the minimum corner just "outside" the half space.
      // Note: It's unclear why this particular test requires an epsilon twice
      // as large (compared to the other tests) to work across all
      // configurations of R_CH.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmin_C(2) + 2 * kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_FALSE(Aabb::HasOverlap(aabb_H, hs_C, X_CH));
    }
    {
      // Place the minimum corner just "below" the plane.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmin_C(2) - kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Aabb::HasOverlap(aabb_H, hs_C, X_CH));
    }

    // As soon as the box penetrates the half space, no amount of movement
    // against the surface normal direction will ever report a non-overlapping
    // state. We sample from that sub-domain by pushing the maximum corner
    // (Vmax) near the boundary and then push the whole box deep into the
    // half space.
    //
    // Vmax is the reflection of Vmin over Bo (the origin of the box). We'll
    // express all vectors in the C frame so we can place that corner just above
    // and below the z = 0 plane in Frame C using the same trick as documented
    // above.
    const Vector3d p_HoBo_C = R_CH * p_HoBo_H;
    const Vector3d p_HoVmax_C = p_HoVmin_C + 2 * (p_HoBo_C - p_HoVmin_C);
    {
      // Put the maximum corner just above the z = 0 plane in Frame C.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmax_C(2) + kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Aabb::HasOverlap(aabb_H, hs_C, X_CH));
    }

    {
      // Put the maximum corner just below the z = 0 plane in Frame C.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmax_C(2) - kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Aabb::HasOverlap(aabb_H, hs_C, X_CH));
    }

    {
      // Bury the box deep in the half space.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmax_C(2) - 1e8}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Aabb::HasOverlap(aabb_H, hs_C, X_CH));
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
