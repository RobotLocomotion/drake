#pragma once

#include <utility>

#include "drake/geometry/proximity/mesh_deformer.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): When SceneGraph properly supports deformable meshes
//  in a public manner, move this out of internal and into geometry along with
//  VolumeMesh.
/* Representation of a volume mesh whose vertices can be moved.

 This is currently a speculative feature and not formally supported by
 SceneGraph. When SceneGraph formally supports deformable geometry, this will
 come out of the `internal` namespace. */
template <typename T>
class DeformableVolumeMesh {
 public:
  /* Constructs a deformable representation of the given mesh whose vertex
   positions are measured and expressed in Frame M. When the vertex positions
   are updated, those values must likewise be measured and expressed in this
   same frame.

   The %DeformableVolumeMesh will *own* its own underlying VolumeMesh. */
  explicit DeformableVolumeMesh(VolumeMesh<T> mesh_M)
      : mesh_(std::move(mesh_M)),
        deformer_(&mesh_) {}

  /* @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{

  DeformableVolumeMesh(const DeformableVolumeMesh& other)
      : DeformableVolumeMesh(other.mesh()) {}

  DeformableVolumeMesh& operator=(const DeformableVolumeMesh& other) {
    if (this == &other) return *this;
    mesh_ = other.mesh();
    return *this;
  }

  // Note: MeshDeformer has no move or copy semantics, so we can't default
  //  semantics here.

  DeformableVolumeMesh(DeformableVolumeMesh&& other)
      : mesh_(std::move(other.mesh_)), deformer_(&mesh_) {}

  DeformableVolumeMesh& operator=(DeformableVolumeMesh&& other) {
    mesh_ = std::move(other.mesh_);
    return *this;
  }

  //@}

  /* Access to the underlying mesh. Represent the most recent configuration
   based either on construction or a call to UpdateVertexPositions(). */
  const VolumeMesh<T>& mesh() const { return mesh_; }

  /* Updates the vertex positions of the underlying mesh.

  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size == 3 * mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<T>>& q);

 private:
  VolumeMesh<T> mesh_;
  MeshDeformer<VolumeMesh<T>> deformer_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
