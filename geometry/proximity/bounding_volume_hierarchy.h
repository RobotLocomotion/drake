#pragma once

#include <algorithm>
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
struct AABB {
  /** Center point of the box.  */
  Vector3<double> center;
  /** Halfwidth extents along each axes.  */
  Vector3<double> halfwidth;

  /** @return Volume of the bounding box.  */
  int volume() {
    return halfwidth[0] * 2 * halfwidth[1] * 2 * halfwidth[2] * 2;
  }
};

/** Node of the tree structure representing the BoundingVolumeHierarchy.  */
template <class MeshType>
struct BVNode {
  AABB aabb;
  BVNode<MeshType>* left;
  BVNode<MeshType>* right;
  /** Leaf node's link to the actual mesh element.  */
  typename MeshType::ElementIndex e;
};

/** BoundingVolumeHierarchy groups mesh elements within bounding volumes to
 support broad-phase operations. The underlying structure is a binary tree of
 axis-aligned bounding boxes (AABB). Leaf nodes contain an index into elements
 of the mesh. The BVH needs a reference to the mesh in order to build the tree,
 but does not own the mesh. It currently assumes that the mesh remains static
 and is not mutable.
*/
template <class MeshType>
class BoundingVolumeHierarchy {
 public:
  using CentroidPair =
      std::pair<typename MeshType::ElementIndex, Vector3<double>>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BoundingVolumeHierarchy)

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

  /** Copy to a new %BoundingVolumeHierarchy and set the new
   %BoundingVolumeHierarchy to use a new compatible mesh.
   %BoundingVolumeHierarchy needs a mesh to operate; however,
   %BoundingVolumeHierarchy does not own the mesh. In fact, several
   %BoundingVolumeHierarchy objects can use the same mesh.
   */
  DRAKE_NODISCARD std::unique_ptr<BoundingVolumeHierarchy> CloneAndSetMesh(
      const MeshType* new_mesh) const {
    DRAKE_DEMAND(new_mesh != nullptr);
    DRAKE_DEMAND(new_mesh->num_vertices() == mesh_->num_vertices());
    std::unique_ptr<BoundingVolumeHierarchy> new_bvh = CloneWithNullMesh();
    new_bvh->mesh_ = new_mesh;
    // TODO(tehbelinda): Recalculate the bounding volume tree for the new mesh
    // when we allow mutation.
    return new_bvh;
  }

  // Clones BoundingVolumeHierarchy data under the assumption that the mesh
  // pointer is null.
  DRAKE_NODISCARD std::unique_ptr<BoundingVolumeHierarchy> CloneWithNullMesh()
      const {
    return std::make_unique<BoundingVolumeHierarchy>(*this);
  }

  const MeshType& mesh() const { return *mesh_; }

 private:
  // Convenience class for testing.
  friend class BVHTester;

  // We use `reset_on_copy` so that the default copy constructor resets
  // the pointer to null when a BoundingVolumeHierarchy is copied.
  reset_on_copy<const MeshType*> mesh_;

  const int num_vertices_per_element_;

  BVNode<MeshType>* bv_tree_;

  BVNode<MeshType>* BuildBVTree(
      typename std::vector<CentroidPair>::iterator start,
      typename std::vector<CentroidPair>::iterator end) {
    // Generate bounding volume.
    AABB aabb = ComputeBoundingVolume(start, end);

    BVNode<MeshType>* node = new BVNode<MeshType>({aabb});
    int num_elements = end - start;
    if (num_elements == 1) {
      // Store element index in this leaf node.
      node->e = start->first;
    } else {
      // Sort by centroid along the x-axis for now.
      // TODO(tehbelinda): Use a better method for splitting, e.g. checking all
      // axes / surface area / etc.
      std::sort(start, end, [](CentroidPair& a, CentroidPair& b) {
        return a.second[0] < b.second[0];
      });

      // Continue with the next branches.
      typename std::vector<CentroidPair>::iterator mid =
          start + num_elements / 2;
      node->left = BuildBVTree(start, mid);
      node->right = BuildBVTree(mid, end);
    }
    return node;
  }

  AABB ComputeBoundingVolume(
      typename std::vector<CentroidPair>::iterator start,
      typename std::vector<CentroidPair>::iterator end) const {
    // Keep track of the min/max bounds to create the bounding box.
    double min_limit = std::numeric_limits<double>::lowest();
    Vector3<double> max_bounds =
        Vector3<double>{min_limit, min_limit, min_limit};
    double max_limit = std::numeric_limits<double>::max();
    Vector3<double> min_bounds =
        Vector3<double>{max_limit, max_limit, max_limit};

    // Check each mesh element in the given range.
    for (auto pair = start; pair < end; ++pair) {
      const auto& element = mesh_->element(pair->first);
      // Check each vertex in the element.
      for (int v = 0; v < num_vertices_per_element_; ++v) {
        const auto& vertex = mesh_->vertex(element.vertex(v)).r_MV();
        // Compare its extent along each of the 3 axes.
        for (int axis = 0; axis < 3; ++axis) {
          if (vertex[axis] < min_bounds[axis]) {
            min_bounds[axis] = vertex[axis];
          }
          if (vertex[axis] > max_bounds[axis]) {
            max_bounds[axis] = vertex[axis];
          }
        }
      }
    }
    Vector3<double> center = (min_bounds + max_bounds) / 2;
    Vector3<double> halfwidth = max_bounds - center;
    return AABB{center, halfwidth};
  }

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
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
