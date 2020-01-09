#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <stack>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_on_copy.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/** Axis-aligned bounding box used in BoundingVolumeHierarchy.  */
class Aabb {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Aabb)

  /**
   @pre half_width.x(), half_width.y(), half_width.z() are not negative.
  */
  Aabb(Vector3<double> center, Vector3<double> half_width)
      : center_(std::move(center)), half_width_(std::move(half_width)) {
    DRAKE_DEMAND(half_width.x() >= 0.0);
    DRAKE_DEMAND(half_width.y() >= 0.0);
    DRAKE_DEMAND(half_width.z() >= 0.0);

    PadBoundary();
  }

  /** Returns the center. */
  const Vector3<double>& center() const { return center_; }

  /** Returns the half_width. */
  const Vector3<double>& half_width() const { return half_width_; }

  /** Returns the upper bounding point. */
  Vector3<double> upper() const { return center_ + half_width_; }

  /** Returns the lower bounding point. */
  Vector3<double> lower() const { return center_ - half_width_; }

  /** @return Volume of the bounding box.  */
  double CalcVolume() const {
    // Double the three half widths using * 8 instead of repeating * 2 three
    // times to help the compiler out.
    return half_width_[0] * half_width_[1] * half_width_[2] * 8;
  }

  /** Checks whether the two bounding volumes overlap by applying the transform
   between the two boxes and using Gottschalk's OBB overlap test.  */
  static bool HasOverlap(const Aabb& a, const Aabb& b,
                         const math::RigidTransform<double>& X_AB);

 private:
  friend class AabbTester;

  // Pad this box in place by a small amount to ensure there will be no
  // roundoff problems. The amount to pad depends on the default tolerance for
  // this precision, the dimensions, and the position of the box in space.
  // A very large box, or a box that is very far from the origin, must be
  // padded more than a small one at the origin.
  void PadBoundary();

  // Default tolerance for double precision. This is the minimum amount of
  // padding to be added to the boundary, regardless of size or position.
  static constexpr double kTolerance = 2e-14;

  // Center point of the box.
  Vector3<double> center_;
  // Half width extents along each axes.
  Vector3<double> half_width_;
};

/** Node of the tree structure representing the BoundingVolumeHierarchy.  */
template <class MeshType>
class BvNode {
 public:
  /** Constructor for leaf nodes.
   @param aabb The bounding volume encompassing the element.
   @param index The index into the mesh for retrieving the element.
   */
  BvNode(Aabb aabb, typename MeshType::ElementIndex index)
      : aabb_(std::move(aabb)), child_(index) {}

  /** Constructor for branch/internal nodes.
   @param aabb The bounding volume encompassing the elements in child branches.
   @param left Unique pointer to the left child branch.
   @param right Unique pointer to the right child branch.
   @pre Both children must be distinct and not null.
   */
  BvNode(Aabb aabb, std::unique_ptr<BvNode<MeshType>> left,
         std::unique_ptr<BvNode<MeshType>> right)
      : aabb_(std::move(aabb)),
        child_(NodeChildren(std::move(left), std::move(right))) {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BvNode)

  /** Returns the bounding volume.  */
  const Aabb& aabb() const { return aabb_; }

  /** Returns the index into the mesh's elements.
   @pre Assumes that is_leaf() returns true.  */
  typename MeshType::ElementIndex element_index() const {
    return std::get<typename MeshType::ElementIndex>(child_);
  }

  /** Returns the left child branch.
   @pre Assumes that is_leaf() returns false.  */
  const BvNode<MeshType>& left() const {
    return *(std::get<NodeChildren>(child_).left);
  }

  /** Returns the right child branch.
   @pre Assumes that is_leaf() returns false.  */
  const BvNode<MeshType>& right() const {
    return *(std::get<NodeChildren>(child_).right);
  }

  /** Returns whether this is a leaf node as a opposed to a branch node.  */
  bool is_leaf() const {
    return std::holds_alternative<typename MeshType::ElementIndex>(child_);
  }

 private:
  struct NodeChildren {
    std::unique_ptr<BvNode<MeshType>> left;
    std::unique_ptr<BvNode<MeshType>> right;

    NodeChildren(std::unique_ptr<BvNode<MeshType>> left_in,
                 std::unique_ptr<BvNode<MeshType>> right_in)
        : left(std::move(left_in)), right(std::move(right_in)) {
      DRAKE_DEMAND(left != nullptr);
      DRAKE_DEMAND(right != nullptr);
      DRAKE_DEMAND(left != right);
    }

    NodeChildren(const NodeChildren& other)
        : NodeChildren{std::make_unique<BvNode<MeshType>>(*other.left),
                       std::make_unique<BvNode<MeshType>>(*other.right)} {}

    NodeChildren& operator=(NodeChildren other) {
      std::swap(left, other.left);
      std::swap(right, other.right);
      return *this;
    }
  };

  Aabb aabb_;

  // If this is a leaf node then the child refers to an index into the mesh's
  // elements (i.e., a tri or a tet) bounded by the node's bounding volume.
  // Otherwise, it refers to child nodes further down the tree.
  std::variant<typename MeshType::ElementIndex, NodeChildren> child_;
};

/** Resulting instruction from performing the bounding volume tree traversal
 (BVTT) callback on two potentially colliding pairs. Note that this is not the
 mathematical result but information on how the traversal should proceed.  */
enum BvttCallbackResult {
  Continue,
  Terminate
};

/** Bounding volume tree traversal (BVTT) callback. Returns a BvttCallbackResult
 for further action, e.g. deciding whether to exit early.  */
template <class MeshType, class OtherMeshType>
using BvttCallback = std::function<BvttCallbackResult(
    typename MeshType::ElementIndex, typename OtherMeshType::ElementIndex)>;

/** BoundingVolumeHierarchy is an acceleration structure for performing spatial
 queries against a collection of objects (in this case, triangles or
 tetrahedra). Specifically, for identifying those objects in or near a
 particular region of interest. It serves as a basis for culling objects that
 are trivially far from the region, reducing the number of "narrow-phase"
 calculations. The underlying structure is a binary tree of axis-aligned
 bounding boxes (Aabb) that encompass one or more mesh elements. Leaf nodes
 contain a single element index into elements of the mesh. The BVH needs a
 reference to the mesh in order to build the tree, but does not own the mesh.
 @pre    Assumes that the mesh is not mutable. Modifications to the mesh after
         constructing the BVH will make the BVH invalid.
 @tparam MeshType SurfaceMesh<double> or VolumeMesh<double> (Exotic types like
         SurfaceMesh<AutoDiffXd> are not supported).
*/
template <class MeshType>
class BoundingVolumeHierarchy {
 public:
  using IndexType = typename MeshType::ElementIndex;

  explicit BoundingVolumeHierarchy(const MeshType& mesh);

  BoundingVolumeHierarchy(const BoundingVolumeHierarchy& bvh) { *this = bvh; }

  BoundingVolumeHierarchy& operator=(const BoundingVolumeHierarchy& bvh) {
    if (&bvh == this) return *this;

    root_node_ = std::make_unique<BvNode<MeshType>>(*bvh.root_node_);
    return *this;
  }

  BoundingVolumeHierarchy(BoundingVolumeHierarchy&&) = default;
  BoundingVolumeHierarchy& operator=(BoundingVolumeHierarchy&&) = default;

  const BvNode<MeshType>& root_node() const { return *root_node_; }

  /** Perform a query of this bvh's mesh elements against the given bvh's
   mesh elements and runs the callback for each unculled pair.  */
  template <class OtherMeshType>
  void Collide(const BoundingVolumeHierarchy<OtherMeshType>& bvh,
               const math::RigidTransform<double>& X_AB,
               BvttCallback<MeshType, OtherMeshType> callback) const {
    using NodePair =
        std::pair<const BvNode<MeshType>&, const BvNode<OtherMeshType>&>;
    std::stack<NodePair, std::vector<NodePair>> node_pairs;
    node_pairs.emplace(root_node(), bvh.root_node());

    while (!node_pairs.empty()) {
      const auto& [node_a, node_b] = node_pairs.top();
      node_pairs.pop();

      // Check if the bounding volumes overlap.
      if (!Aabb::HasOverlap(node_a.aabb(), node_b.aabb(), X_AB)) {
        continue;
      }

      // Run the callback on the pair if they are both leaf nodes, otherwise
      // check each branch.
      if (node_a.is_leaf() && node_b.is_leaf()) {
        BvttCallbackResult result =
            callback(node_a.element_index(), node_b.element_index());
        if (result == BvttCallbackResult::Terminate) return;  // Exit early.
      } else if (node_b.is_leaf()) {
        node_pairs.emplace(node_a.left(), node_b);
        node_pairs.emplace(node_a.right(), node_b);
      } else if (node_a.is_leaf()) {
        node_pairs.emplace(node_a, node_b.left());
        node_pairs.emplace(node_a, node_b.right());
      } else {
        node_pairs.emplace(node_a.left(), node_b.left());
        node_pairs.emplace(node_a.right(), node_b.left());
        node_pairs.emplace(node_a.left(), node_b.right());
        node_pairs.emplace(node_a.right(), node_b.right());
      }
    }
  }

  /** Wrapper around `Collide` with a callback that accumulates each pair of
   collision candidates and returns them all.
   @return Vector of element index pairs whose bounding volumes are candidates
   for collision.  */
  template <class OtherMeshType>
  std::vector<std::pair<IndexType, typename OtherMeshType::ElementIndex>>
  GetCollisionCandidates(const BoundingVolumeHierarchy<OtherMeshType>& bvh,
                         const math::RigidTransform<double>& X_AB) const {
    std::vector<std::pair<IndexType, typename OtherMeshType::ElementIndex>>
        result;
    auto callback =
        [&result](
            IndexType a,
            typename OtherMeshType::ElementIndex b) -> BvttCallbackResult {
      result.emplace_back(a, b);
      return BvttCallbackResult::Continue;
    };
    Collide(bvh, X_AB, callback);
    return result;
  }

 private:
  // Convenience class for testing.
  friend class BVHTester;

  using CentroidPair = std::pair<IndexType, Vector3<double>>;

  static std::unique_ptr<BvNode<MeshType>> BuildBVTree(
      const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator& start,
      const typename std::vector<CentroidPair>::iterator& end);

  static Aabb ComputeBoundingVolume(
      const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator& start,
      const typename std::vector<CentroidPair>::iterator& end);

  // TODO(tehbelinda): Move this funtion into SurfaceMesh/VolumeMesh directly
  // and rename to CalcElementCentroid(ElementIndex).
  static Vector3<double> ComputeCentroid(const MeshType& mesh,
                                         IndexType i);

  static constexpr int kElementVertexCount = MeshType::kDim + 1;

  std::unique_ptr<BvNode<MeshType>> root_node_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

extern template class drake::geometry::internal::BoundingVolumeHierarchy<
    drake::geometry::SurfaceMesh<double>>;
extern template class drake::geometry::internal::BoundingVolumeHierarchy<
    drake::geometry::VolumeMesh<double>>;
