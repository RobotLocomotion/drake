#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_nodiscard.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_on_copy.h"

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

/** Node of the tree structure representing the BoundingVolumeHierarchy.  */
template <class MeshType>
struct BvNode {
  explicit BvNode(Aabb aabb_in) : aabb(std::move(aabb_in)) {}

  BvNode(const BvNode& node) : aabb(node.aabb) { *this = node; }

  BvNode& operator=(const BvNode& node) {
    if (&node == this) return *this;

    aabb = node.aabb;
    element_index = node.element_index;
    if (node.left != nullptr) {
      left = std::make_unique<BvNode<MeshType>>(*node.left);
    }
    if (node.right != nullptr) {
      right = std::make_unique<BvNode<MeshType>>(*node.right);
    }
    return *this;
  }

  BvNode(BvNode&&) = default;
  BvNode& operator=(BvNode&&) = default;

  Aabb aabb;
  /** Leaf node's index to the mesh element (i.e., tri or tet) bounded by the
   node's bounding volume.  */
  typename MeshType::ElementIndex element_index;
  std::unique_ptr<BvNode<MeshType>> left;
  std::unique_ptr<BvNode<MeshType>> right;
};

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
  explicit BoundingVolumeHierarchy(const MeshType& mesh) {
    // Generate element indices and corresponding centroids. These are used
    // for calculating the split point of the volumes.
    const int num_elements = mesh.num_elements();
    std::vector<CentroidPair> element_centroids;
    for (typename MeshType::ElementIndex i(0); i < num_elements; ++i) {
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

 private:
  // Convenience class for testing.
  friend class BVHTester;

  using CentroidPair =
      std::pair<typename MeshType::ElementIndex, Vector3<double>>;

  static std::unique_ptr<BvNode<MeshType>> BuildBVTree(const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator start,
      const typename std::vector<CentroidPair>::iterator end) {
    // Generate bounding volume.
    const Aabb aabb = ComputeBoundingVolume(mesh, start, end);

    auto node = std::make_unique<BvNode<MeshType>>(aabb);
    const int num_elements = end - start;
    if (num_elements == 1) {
      // Store element index in this leaf node.
      node->element_index = start->first;
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
      node->left = BuildBVTree(mesh, start, mid);
      node->right = BuildBVTree(mesh, mid, end);
    }
    return node;
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
      const typename MeshType::ElementIndex i) {
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

  static constexpr int kElementVertexCount = MeshType::kDim + 1;

  std::unique_ptr<BvNode<MeshType>> root_node_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
