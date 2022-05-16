#include "drake/geometry/proximity/bvh_updater.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/mesh_deformer.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RotationMatrix;
using std::vector;

template <class MeshType>
class BvhUpdaterTest : public ::testing::Test {
 protected:
  /* Creates a generic mesh of two disjoint elements as shown below.

                          Az    Ay
                 v5       ┆   ╱
                  ●       ┆  ╱    ● v2
                  ┆       ┆ ╱     ┆
          v4      ┆v7     ┆╱    v6┆       v1
    ┄┄┄┄┄┄┄●┄┄┄┄┄┄●┄┄┄┄┄┄┄┼┄┄┄┄┄┄┄●┄┄┄┄┄┄●┄ Ax
                 ╱       ╱┆      ╱
                ╱       ╱ ┆     ╱
               ●       ╱  ┆    ● v0
              V3      ╱   ┆


    If the MeshType is a surface mesh, triangles (0, 1, 2) and (3, 4, 5) are
    created. If MeshType is a volume mesh, tetrahedra (0, 1, 2, 6) and
    (3, 4, 5, 7) are created. The axis aligned boxes for both of those element
    pairs will be the same.

    We're not going to worry about the mesh having "too many" vertices for a
    surface mesh; they won't affect the Bvh.

    The goal is to create an Aabb Bvh which will have a single root node and
    two leaf nodes (see the UpdateNode test below). The Bvh can be configured
    to contain an *arbitrary* number of elements in the leaf nodes. To guarantee
    the desired Bvh structure, we'll duplicate each element enough times to
    produce two full leaf nodes (with identical elements).

    @param dist  The distance from the origin to the "origin" of the elements
                 (i.e., vertices 6 and 7).  */
  static MeshType MakeMesh(double dist = 2) {
    using T = typename MeshType::ScalarType;
    const Vector3<T> offset{dist, 0, 0};
    // clang-format off
    vector<Vector3<T>> vertices{
        {Vector3<T>{0, -1, 0} + offset,
         Vector3<T>{1, 0, 0} + offset,
         Vector3<T>{0, 0, 1} + offset,
         Vector3<T>{0, -1, 0} - offset,
         Vector3<T>{-1, 0, 0} - offset,
         Vector3<T>{0, 0, 1} - offset,
         offset,
         -offset}};
    // clang-format on
    if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
      /* The winding of the triangles don't matter. */
      vector<SurfaceTriangle> triangles;
      for (int i = 0; i < MeshTraits<MeshType>::kMaxElementPerBvhLeaf; ++i) {
        triangles.emplace_back(0, 1, 2);
        triangles.emplace_back(3, 4, 5);
      }
      return MeshType(std::move(triangles), std::move(vertices));
    } else {
      /* We don't have to worry about whether the tet is "inside out" or not
       (the 3D version of "winding"). */
      vector<VolumeElement> tets;
      for (int i = 0; i < MeshTraits<MeshType>::kMaxElementPerBvhLeaf; ++i) {
        tets.emplace_back(0, 1, 2, 6);
        tets.emplace_back(3, 4, 5, 7);
      }
      return MeshType(std::move(tets), std::move(vertices));
    }
  }
};

using MeshTypes = ::testing::Types<TriangleSurfaceMesh<double>,
                                   TriangleSurfaceMesh<AutoDiffXd>,
                                   VolumeMesh<double>, VolumeMesh<AutoDiffXd>>;

TYPED_TEST_SUITE(BvhUpdaterTest, MeshTypes);

/* Tests construction logic for the combination of mesh type and scalar type.
 Confirms direct and move constructors (copy constructors have been disabled).
 */
TYPED_TEST(BvhUpdaterTest, Construction) {
  using MeshType = TypeParam;
  const MeshType mesh = this->MakeMesh();
  Bvh<Aabb, MeshType> bvh(mesh);
  BvhUpdater<MeshType> updater(&mesh, &bvh);
  EXPECT_EQ(&updater.mesh(), &mesh);
  EXPECT_EQ(&updater.bvh(), &bvh);
}

/* Tests that the updater refits the bounding volumes as expected. Using the
 mesh created by MakeMesh(), we can easily predict the expected bounding boxes.
 We apply an arbitrary transformation to the vertices and confirm that the
 bounding boxes change as expected. */
TYPED_TEST(BvhUpdaterTest, Update) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  const double dist = 3;
  MeshType mesh = this->MakeMesh(dist);
  Bvh<Aabb, MeshType> bvh(mesh);
  BvhUpdater<MeshType> updater(&mesh, &bvh);
  MeshDeformer<MeshType> deformer(&mesh);

  /* First, confirm the initial conditions:
    - Topology of the BVH (a root with two leaves)
    - Domains of the three bounding volumes */

  /* Quick test of the topology */
  const auto& root = bvh.root_node();
  ASSERT_FALSE(root.is_leaf());
  ASSERT_TRUE(root.left().is_leaf());
  ASSERT_TRUE(root.right().is_leaf());

  const auto& left = root.left();
  const auto& right = root.right();

  const Aabb expected_root_bv(Vector3d{0, -0.5, 0.5},
                              Vector3d{dist + 1, 0.5, 0.5});
  const Aabb expected_left_bv(Vector3d{-dist - 0.5, -0.5, 0.5},
                              Vector3d{0.5, 0.5, 0.5});
  const Aabb expected_right_bv(Vector3d{dist + 0.5, -0.5, 0.5},
                               Vector3d{0.5, 0.5, 0.5});

  /* Test the domains of the bounding boxes. */
  ASSERT_TRUE(CompareMatrices(root.bv().center(), expected_root_bv.center()));
  ASSERT_TRUE(
      CompareMatrices(root.bv().half_width(), expected_root_bv.half_width()));

  ASSERT_TRUE(CompareMatrices(left.bv().center(), expected_left_bv.center()));
  ASSERT_TRUE(
      CompareMatrices(left.bv().half_width(), expected_left_bv.half_width()));

  ASSERT_TRUE(CompareMatrices(right.bv().center(), expected_right_bv.center()));
  ASSERT_TRUE(
      CompareMatrices(right.bv().half_width(), expected_right_bv.half_width()));

  // TODO(SeanCurtis-TRI): Rather than limiting ourselves to this 90-degree
  // rotation. We can construct a new BVH from a new mesh and compare the
  // updated and freshly constructed bvhs.

  /* Second, deform the mesh; we'll simply rotate all vertices 90 degrees. We
   pick the 90-degree rotation to simplify the logic to know what BV to expect
   at the end. A 90-degree rotation of the vertices essentially mean that the
   bounding boxes will likewise be a 90-degree rotation. That won't be true for
   just about any other angle. */
  RotationMatrix<T> R = RotationMatrix<T>::MakeZRotation(M_PI / 2);

  /* 3 doubles for each of M vertices */
  VectorX<T> p_MVs(3 * mesh.num_vertices());
  for (int i = 0; i < mesh.num_vertices(); ++i) {
    p_MVs.segment(i * 3, 3) << R * mesh.vertex(i);
  }
  deformer.SetAllPositions(p_MVs);
  updater.Update();

  /* Third, compare the updated bounding volumes with the expected boxes. In
   this case, they are a rotation of the original boxes.  The act of rotation
   can introduce epsilon rounding error, so we test with respect to round-off
   scaled tolerance. */
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  EXPECT_TRUE(CompareMatrices(root.bv().center(),
                              R * expected_root_bv.center().cast<T>(), kEps));
  EXPECT_TRUE(CompareMatrices(
      root.bv().half_width(),
      (R * expected_root_bv.half_width().cast<T>()).cwiseAbs(), 2 * kEps));
  EXPECT_TRUE(CompareMatrices(left.bv().center(),
                              R * expected_left_bv.center().cast<T>(), kEps));
  EXPECT_TRUE(CompareMatrices(
      left.bv().half_width(),
      (R * expected_left_bv.half_width().cast<T>()).cwiseAbs(), 2 * kEps));
  EXPECT_TRUE(CompareMatrices(right.bv().center(),
                              R * expected_right_bv.center().cast<T>(), kEps));
  EXPECT_TRUE(CompareMatrices(
      right.bv().half_width(),
      (R * expected_right_bv.half_width().cast<T>()).cwiseAbs(), 2 * kEps));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
