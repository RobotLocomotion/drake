#pragma once

#include <limits>
#include <vector>

#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/bvh.h"

namespace drake {
namespace geometry {
namespace internal {

/* This class can be used to update a bounding-volume hierarchy (BVH). It
 doesn't own the BVH or the corresponding mesh, but merely applies an algorithm
 to the BVH which compares its current configuration against the underlying
 mesh data, updating the BVH state to maintain correct spatial culling.

 This will frequently be combined with a MeshDeformer so that when a mesh is
 updated, the corresponding Bvh can likewise be updated.

 This current incarnation only supports Bvhs constructed with axis-aligned
 bounding boxes.

 This class cannot be moved or copied. It is assumed upon creation that it will
 be permanently associated with *specific* MeshType and Bvh instances and
 maintain those associations for its entire lifetime (see DeformableVolumeMesh
 as an example).

 @tparam MeshType TriangleSurfaceMesh<T> or VolumeMesh<T> where T is double or
                  AutoDiffXd. */
template <typename MeshType>
class BvhUpdater {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BvhUpdater)

  /* Constructs a %BvhUpdater for the given BVH and its corresponding mesh.
   Both the mesh and the bvh must remain alive at least as long as this
   updater.

   @param mesh_M   The underlying mesh, measured and expressed in Frame M.
   @param bvh_M    The Bvh for the mesh, likewise measured and expressed in the
                   mesh's frame M.
   @pre bvh_M was constructed on mesh_M.
   @pre mesh_M != nullptr and bvh_M != nullptr. */
  BvhUpdater(const MeshType* mesh_M, Bvh<Aabb, MeshType>* bvh_M)
      : mesh_(*mesh_M), bvh_(*bvh_M) {
    DRAKE_DEMAND(mesh_M != nullptr);
    DRAKE_DEMAND(bvh_M != nullptr);
  }

  const MeshType& mesh() const { return mesh_; }
  const Bvh<Aabb, MeshType>& bvh() const { return bvh_; }

  /* Updates the referenced bvh to maintain a good fit on the referenced mesh.
   */
  void Update() {
    /* Get the *double-valued* mesh vertices. */
    const auto& vertices = GetMeshVertices(mesh_.vertices());
    if (vertices.size() == 0) return;

    /* This implementation doesn't change the bvh topology; it simply passes
     through each box in a bottom-up manner refitting the box to the data. */
    UpdateRecursive(&bvh_.mutable_root_node(), vertices);
  }

 private:
  // If the mesh type is already double-valued, simply return the mesh vertices.
  static const std::vector<Vector3<double>>& GetMeshVertices(
      const std::vector<Vector3<double>>& vertices) {
    return vertices;
  }

  // If the mesh type is AutoDiffXd-valued, return double-valued vertices.
  static std::vector<Vector3<double>> GetMeshVertices(
      const std::vector<Vector3<AutoDiffXd>>& vertices) {
    std::vector<Vector3<double>> vertices_dbl;
    vertices_dbl.reserve(vertices.size());
    for (const auto& v : vertices) {
      vertices_dbl.emplace_back(convert_to_double(v));
    }
    return vertices_dbl;
  }

  // Helper function to perform a bottom-up refit.
  void UpdateRecursive(typename Bvh<Aabb, MeshType>::NodeType* node,
                       const std::vector<Vector3<double>>& vertices) {
    using Vector3d = Eigen::Vector3d;
    /* Intentionally uninitialized. */
    Vector3d lower, upper;
    constexpr int kElementVertexCount = MeshType::kVertexPerElement;
    constexpr double kInf = std::numeric_limits<double>::infinity();
    if (node->is_leaf()) {
      // TODO(SeanCurtis-TRI): This is the limiting factor on supporting Obb.
      //  This functionality needs to be a function of the bounding volume type
      //  and not encoded in this class.
      lower << kInf, kInf, kInf;
      upper = -lower;
      const int num_elements = node->num_element_indices();
      for (int e = 0; e < num_elements; ++e) {
        const auto& element = mesh_.element(node->element_index(e));
        for (int i = 0; i < kElementVertexCount; ++i) {
          const Vector3d& p_MV = convert_to_double(vertices[element.vertex(i)]);
          lower = lower.cwiseMin(p_MV);
          upper = upper.cwiseMax(p_MV);
        }
      }
    } else {
      UpdateRecursive(&node->left(), vertices);
      UpdateRecursive(&node->right(), vertices);
      // Update box on child boxes.
      lower = node->left().bv().lower().cwiseMin(node->right().bv().lower());
      upper = node->left().bv().upper().cwiseMax(node->right().bv().upper());
    }
    node->bv().set_bounds(lower, upper);
  }

  const MeshType& mesh_;
  Bvh<Aabb, MeshType>& bvh_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
