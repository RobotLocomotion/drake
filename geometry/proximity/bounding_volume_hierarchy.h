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
#include "drake/common/drake_nodiscard.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_on_copy.h"
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
  Aabb(const Vector3<double> center, const Vector3<double> half_width)
      : center_(std::move(center)), half_width_(std::move(half_width)) {
    // TODO(tehbelinda): Introduce epsilon tolerance for robust bounding box.
    DRAKE_DEMAND(half_width.x() >= 0.0);
    DRAKE_DEMAND(half_width.y() >= 0.0);
    DRAKE_DEMAND(half_width.z() >= 0.0);
  }

  /** Returns the center. */
  const Vector3<double>& center() const { return center_; }

  /** Returns the half_width. */
  const Vector3<double>& half_width() const { return half_width_; }

  /** Returns the upper bounding point. */
  const Vector3<double> upper() const { return center_ + half_width_; }

  /** Returns the lower bounding point. */
  const Vector3<double> lower() const { return center_ - half_width_; }

  /** @return Volume of the bounding box.  */
  double CalcVolume() const {
    // Double the three half widths using * 8 instead of repeating * 2 three
    // times to help the compiler out.
    return half_width_[0] * half_width_[1] * half_width_[2] * 8;
  }

 private:
  // Center point of the box.
  Vector3<double> center_;
  // Half width extents along each axes.
  Vector3<double> half_width_;
};

/** Forward declaration, see below.  */
template <class MeshType>
class BvNode;

/** Grouping for node children branches.  */
template <class MeshType>
struct NodeChildren {
  std::unique_ptr<BvNode<MeshType>> left;
  std::unique_ptr<BvNode<MeshType>> right;

  /** @pre Both children must always be defined and not null.  */
  NodeChildren(std::unique_ptr<BvNode<MeshType>> left_in,
               std::unique_ptr<BvNode<MeshType>> right_in)
      : left(std::move(left_in)), right(std::move(right_in)) {
    DRAKE_DEMAND(left != nullptr);
    DRAKE_DEMAND(right != nullptr);
    DRAKE_DEMAND(left != right);
  }

  NodeChildren(const NodeChildren& other) { *this = other; }

  NodeChildren& operator=(const NodeChildren& other) {
    if (&other == this) return *this;

    left = std::make_unique<BvNode<MeshType>>(*(other.left));
    right = std::make_unique<BvNode<MeshType>>(*(other.right));
    return *this;
  }

  NodeChildren(NodeChildren&&) = default;
  NodeChildren& operator=(NodeChildren&&) = default;
};

/** Node of the tree structure representing the BoundingVolumeHierarchy.  */
template <class MeshType>
class BvNode {
 public:
  BvNode(Aabb aabb, typename MeshType::ElementIndex index)
      : aabb_(std::move(aabb)), child_(index) {}

  BvNode(Aabb aabb, std::unique_ptr<BvNode<MeshType>> left,
         std::unique_ptr<BvNode<MeshType>> right)
      : aabb_(std::move(aabb)),
        child_(NodeChildren<MeshType>(std::move(left), std::move(right))) {}

  BvNode(const BvNode& node) : aabb_(node.aabb_) {
    *this = node;
  }

  BvNode& operator=(const BvNode& node) {
    if (&node == this) return *this;

    aabb_ = node.aabb_;
    child_ = node.child_;
    return *this;
  }

  BvNode(BvNode&&) = default;
  BvNode& operator=(BvNode&&) = default;

  const Aabb& aabb() const { return aabb_; }

  const typename MeshType::ElementIndex element_index() const {
    return std::get<typename MeshType::ElementIndex>(child_);
  }

  const BvNode<MeshType>& left() const {
    return *(std::get<NodeChildren<MeshType>>(child_).left);
  }

  const BvNode<MeshType>& right() const {
    return *(std::get<NodeChildren<MeshType>>(child_).right);
  }

  bool IsLeaf() const {
    return std::holds_alternative<typename MeshType::ElementIndex>(child_);
  }

 private:
  Aabb aabb_;

  // If this is a leaf node then the child refers to an index into the mesh's
  // elements (i.e., a tri or a tet) bounded by the node's bounding volume.
  // Otherwise, it refers to child nodes further down the tree.
  std::variant<typename MeshType::ElementIndex, NodeChildren<MeshType>> child_;
};

/** Result from performing the bounding volume tree traversal (BVTT) callback
 on two colliding pairs.  */
enum BvttCallbackResult {
  Continue = 0,
  Terminate = 1
};

/** Bounding volume tree traversal callback. Returns a BvttCallbackResult for
 further action, e.g. deciding whether to exit early.  */
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

  explicit BoundingVolumeHierarchy(const MeshType& mesh) {
    // Generate element indices and corresponding centroids. These are used
    // for calculating the split point of the volumes.
    const int num_elements = mesh.num_elements();
    std::vector<CentroidPair> element_centroids;
    for (IndexType i(0); i < num_elements; ++i) {
      element_centroids.emplace_back(i, ComputeCentroid(mesh, i));
    }

    root_node_ =
        BuildBVTree(mesh, element_centroids.begin(), element_centroids.end());
  }

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
   mesh elements and runs the callback for each colliding pair.  */
  template <class OtherMeshType, typename T>
  void Collide(const BoundingVolumeHierarchy<OtherMeshType>& bvh,
               const math::RigidTransform<T>& X_AB,
               BvttCallback<MeshType, OtherMeshType> callback) const {
    // Convert transform since we only have double support.
    const math::RigidTransform<double> X_ABd = convert_to_double(X_AB);

    using NodePair =
        std::pair<const BvNode<MeshType>&, const BvNode<OtherMeshType>&>;
    std::stack<NodePair, std::vector<NodePair>> node_pairs;
    node_pairs.emplace(root_node(), bvh.root_node());

    while (!node_pairs.empty()) {
      auto[node_a, node_b] = node_pairs.top();
      node_pairs.pop();

      // Check if the bounding volumes overlap.
      if (!HasOverlap(node_a.aabb(), node_b.aabb(), X_ABd)) {
        continue;
      }

      // Run the callback on the pair if they are both leaf nodes, otherwise
      // check each branch.
      if (node_a.IsLeaf() && node_b.IsLeaf()) {
        BvttCallbackResult result =
            callback(node_a.element_index(), node_b.element_index());
        if (result == BvttCallbackResult::Terminate) return;  // Exit early.
      } else if (node_b.IsLeaf()) {
        node_pairs.emplace(node_a.left(), node_b);
        node_pairs.emplace(node_a.right(), node_b);
      } else if (node_a.IsLeaf()) {
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

  /** Wrapper around `Collide` with a callback that accumulates each colliding
   pair and returns them all.
   @return Vector of element index pairs whose bounding volumes collide.  */
  template <class OtherMeshType, typename T>
  std::vector<std::pair<IndexType, typename OtherMeshType::ElementIndex>>
  GetCollidingPairs(const BoundingVolumeHierarchy<OtherMeshType>& bvh,
                    const math::RigidTransform<T>& X_AB) const {
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

  static std::unique_ptr<BvNode<MeshType>> BuildBVTree(const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator start,
      const typename std::vector<CentroidPair>::iterator end) {
    // Generate bounding volume.
    const Aabb aabb = ComputeBoundingVolume(mesh, start, end);

    const int num_elements = end - start;
    if (num_elements == 1) {
      // Store element index in this leaf node.
      return std::make_unique<BvNode<MeshType>>(aabb, start->first);

    } else {
      // Sort by centroid along the axis of greatest spread.
      int axis{};
      aabb.half_width().maxCoeff(&axis);
      // TODO(tehbelinda): Use a better heuristic for splitting into branches,
      // e.g. surface area.
      std::sort(start, end,
                [axis](const CentroidPair& a, const CentroidPair& b) {
                  return a.second[axis] < b.second[axis];
                });

      // Continue with the next branches.
      const typename std::vector<CentroidPair>::iterator mid =
          start + num_elements / 2;
      return std::make_unique<BvNode<MeshType>>(
          aabb, BuildBVTree(mesh, start, mid), BuildBVTree(mesh, mid, end));
    }
  }

  static const Aabb ComputeBoundingVolume(
      const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator start,
      const typename std::vector<CentroidPair>::iterator end) {
    // Keep track of the min/max bounds to create the bounding box.
    Vector3<double> max_bounds, min_bounds;
    max_bounds.setConstant(std::numeric_limits<double>::lowest());
    min_bounds.setConstant(std::numeric_limits<double>::max());

    // Check each mesh element in the given range.
    for (auto pair = start; pair < end; ++pair) {
      const auto& element = mesh.element(pair->first);
      // Check each vertex in the element.
      for (int v = 0; v < kElementVertexCount; ++v) {
        const auto& vertex = mesh.vertex(element.vertex(v)).r_MV();
        // Compare its extent along each of the 3 axes.
        min_bounds = min_bounds.cwiseMin(vertex);
        max_bounds = max_bounds.cwiseMax(vertex);
      }
    }
    const Vector3<double> center = (min_bounds + max_bounds) / 2;
    const Vector3<double> half_width = max_bounds - center;
    return Aabb(center, half_width);
  }

  // TODO(tehbelinda): Move this funtion into SurfaceMesh/VolumeMesh directly
  // and rename to CalcElementCentroid(ElementIndex).
  static Vector3<double> ComputeCentroid(const MeshType& mesh,
                                         const IndexType i) {
    Vector3<double> centroid{0, 0, 0};
    const auto& element = mesh.element(i);
    // Calculate average from all vertices.
    for (int v = 0; v < kElementVertexCount; ++v) {
      const auto& vertex = mesh.vertex(element.vertex(v)).r_MV();
      centroid += vertex;
    }
    centroid /= kElementVertexCount;
    return centroid;
  }

  static bool HasOverlap(const Aabb& a, const Aabb& b,
                         const math::RigidTransform<double>& X_AB) {
    // Check whether the two bounding volumes overlap by applying the transform
    // between the two boxes and using Gottschalk's OBB overlap test.
    const Vector3<double>& t = X_AB.translation() - a.center() + b.center();
    const Matrix3<double>& r = X_AB.rotation().matrix();

    // Compute some common subexpressions and add epsilon to counteract
    // arithmetic error, e.g. when two edges are parallel.
    Matrix3<double> abs_r = r;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        abs_r(i, j) = abs(abs_r(i, j)) + std::numeric_limits<double>::epsilon();
      }
    }

    // First category of cases separating along a's axes.
    for (int i = 0; i < 3; ++i) {
      if (abs(t[i]) > a.half_width()[i] +
                      b.half_width()[0] * abs_r(i, 0) +
                      b.half_width()[1] * abs_r(i, 1) +
                      b.half_width()[2] * abs_r(i, 2)) {
        return false;
      }
    }

    // Second category of cases separating along b's axes.
    for (int i = 0; i < 3; ++i) {
      if (abs(t[0] * r(0, i) + t[1] * r(1, i) + t[2] * r(2, i)) >
              b.half_width()[i] +
              a.half_width()[0] * abs_r(0, i) +
              a.half_width()[1] * abs_r(1, i) +
              a.half_width()[2] * abs_r(2, i)) {
        return false;
      }
    }

    // Third category of cases separating along the axes formed from the cross
    // products of a's and b's axes.
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (abs(t[(i + 2) % 3] * r((i + 1) % 3, j) -
                t[(i + 1) % 3] * r((i + 2) % 3, j)) >
            a.half_width()[(i + 1) % 3] * abs_r((i + 2) % 3, j) +
                a.half_width()[(i + 2) % 3] * abs_r((i + 1) % 3, j) +
            b.half_width()[(j + 1) % 3] * abs_r(i, (j + 2) % 3) +
                b.half_width()[(j + 2) % 3] * abs_r(i, (j + 1) % 3)) {
          return false;
        }
      }
    }

    return true;
  }

  static constexpr int kElementVertexCount = MeshType::kDim + 1;

  std::unique_ptr<BvNode<MeshType>> root_node_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
