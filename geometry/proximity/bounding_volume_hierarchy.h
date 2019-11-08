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
class AABB {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AABB)

  /**
   @pre halfwidth.x(), halfwidth.y(), halfwidth.z() are not negative.
  */
  AABB(const Vector3<double> center, const Vector3<double> halfwidth)
      : center_(std::move(center)), halfwidth_(std::move(halfwidth)) {
    DRAKE_DEMAND(halfwidth.x() >= 0.0);
    DRAKE_DEMAND(halfwidth.y() >= 0.0);
    DRAKE_DEMAND(halfwidth.z() >= 0.0);
  }

  /** Returns the center. */
  const Vector3<double>& center() const { return center_; }

  /** Returns the halfwidth. */
  const Vector3<double>& halfwidth() const { return halfwidth_; }

  /** @return Volume of the bounding box.  */
  double CalcVolume() {
    return halfwidth_[0] * 2 * halfwidth_[1] * 2 * halfwidth_[2] * 2;
  }

 private:
  /** Center point of the box.  */
  Vector3<double> center_;
  /** Halfwidth extents along each axes.  */
  Vector3<double> halfwidth_;
};

/** Node of the tree structure representing the BoundingVolumeHierarchy.  */
template <class MeshType>
struct BVNode {
  AABB aabb;
  std::unique_ptr<BVNode<MeshType>> left;
  std::unique_ptr<BVNode<MeshType>> right;
  /** Leaf node's link to the actual mesh element.  */
  typename MeshType::ElementIndex e;
};

/** BoundingVolumeHierarchy groups mesh elements within bounding volumes to
 support broad-phase operations. The underlying structure is a binary tree of
 axis-aligned bounding boxes (AABB). Leaf nodes contain an index into elements
 of the mesh. The BVH needs a reference to the mesh in order to build the tree,
 but does not own the mesh.
 @pre    Assumes that the mesh remains static and is not mutable.
 @tparam MeshType SurfaceMesh<double> or VolumeMesh<double> (Exotic types like
         SurfaceMesh<AutoDiffXd> are not supported).
*/
template <class MeshType>
class BoundingVolumeHierarchy {
 public:
  using CentroidPair =
      std::pair<typename MeshType::ElementIndex, Vector3<double>>;

  BoundingVolumeHierarchy(const BoundingVolumeHierarchy& bvh) { *this = bvh; }

  BoundingVolumeHierarchy& operator=(const BoundingVolumeHierarchy& bvh) {
    if (&bvh == this) return *this;

    num_vertices_per_element_ = bvh.num_vertices_per_element_;
    // TODO(tehbelinda): Decide on copy semantics such as deep copy support.
    mesh_ = nullptr;
    bv_tree_ = nullptr;

    return *this;
  }

  BoundingVolumeHierarchy(BoundingVolumeHierarchy&&) = default;
  BoundingVolumeHierarchy& operator=(BoundingVolumeHierarchy&&) = default;

  explicit BoundingVolumeHierarchy(const MeshType* mesh)
      : mesh_(mesh), num_vertices_per_element_(MeshType::kDim + 1) {
    DRAKE_DEMAND(mesh_ != nullptr);

    // Generate element indices and corresponding centroids. These are used
    // for calculating the split point of the volumes.
    const int num_elements = mesh_->num_elements();
    std::vector<CentroidPair> element_centroids;
    for (typename MeshType::ElementIndex i(0); i < num_elements; ++i) {
      Vector3<double> centroid = ComputeCentroid(i);
      element_centroids.push_back(std::pair(i, centroid));
    }

    bv_tree_ = BuildBVTree(element_centroids.begin(), element_centroids.end());
  }

  const MeshType& mesh() const { return *mesh_; }

 private:
  // Convenience class for testing.
  friend class BVHTester;

  std::unique_ptr<BVNode<MeshType>> BuildBVTree(
      const typename std::vector<CentroidPair>::iterator start,
      const typename std::vector<CentroidPair>::iterator end) {
    // Generate bounding volume.
    const AABB aabb = ComputeBoundingVolume(start, end);

    auto node = std::make_unique<BVNode<MeshType>>(BVNode<MeshType>({aabb}));
    const int num_elements = end - start;
    if (num_elements == 1) {
      // Store element index in this leaf node.
      node->e = start->first;
    } else {
      // Sort by centroid along the axis of greatest spread.
      int axis;
      aabb.halfwidth().maxCoeff(&axis);
      // TODO(tehbelinda): Use a better heuristic for splitting into branches,
      // e.g. surface area.
      std::sort(start, end,
                [axis](const CentroidPair& a, const CentroidPair& b) {
                  return a.second[axis] < b.second[axis];
                });

      // Continue with the next branches.
      const typename std::vector<CentroidPair>::iterator mid =
          start + num_elements / 2;
      node->left = BuildBVTree(start, mid);
      node->right = BuildBVTree(mid, end);
    }
    return node;
  }

  const AABB ComputeBoundingVolume(
      const typename std::vector<CentroidPair>::iterator start,
      const typename std::vector<CentroidPair>::iterator end) const {
    // Keep track of the min/max bounds to create the bounding box.
    Vector3<double> max_bounds =
        std::numeric_limits<double>::lowest() * Vector3<double>::Ones();
    Vector3<double> min_bounds =
        std::numeric_limits<double>::max() * Vector3<double>::Ones();

    // Check each mesh element in the given range.
    for (auto pair = start; pair < end; ++pair) {
      const auto& element = mesh_->element(pair->first);
      // Check each vertex in the element.
      for (int v = 0; v < num_vertices_per_element_; ++v) {
        const auto& vertex = mesh_->vertex(element.vertex(v)).r_MV();
        // Compare its extent along each of the 3 axes.
        min_bounds = min_bounds.cwiseMin(vertex);
        max_bounds = max_bounds.cwiseMax(vertex);
      }
    }
    const Vector3<double> center = (min_bounds + max_bounds) / 2;
    const Vector3<double> halfwidth = max_bounds - center;
    return AABB(center, halfwidth);
  }

  // TODO(tehbelinda): Move this funtion into SurfaceMesh/VolumeMesh directly
  // and rename to CalcElementCentroid(ElementIndex).
  Vector3<double> ComputeCentroid(
      const typename MeshType::ElementIndex i) const {
    Vector3<double> centroid{0, 0, 0};
    const auto& element = mesh_->element(i);
    // Calculate average from all vertices.
    for (int v = 0; v < num_vertices_per_element_; ++v) {
      const auto& vertex = mesh_->vertex(element.vertex(v)).r_MV();
      centroid += vertex;
    }
    centroid /= num_vertices_per_element_;
    return centroid;
  }

  // We use `reset_on_copy` so that the default copy constructor resets
  // the pointer to null when a BoundingVolumeHierarchy is copied. This is
  // currently redundant with the explicit null in the current copy constructor
  // (via assignment operator) while we are still working on copy semantics.
  reset_on_copy<const MeshType*> mesh_;

  int num_vertices_per_element_;

  std::unique_ptr<BVNode<MeshType>> bv_tree_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
