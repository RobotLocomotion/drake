#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

/* (Advanced) Class that serves an attorney-client role in allowing disciplined
 deformation of a mesh -- in other words, vertex positions can be changed. The
 topology of the mesh is preserved.

 Deforming a mesh can be a dangerous proposition if that mesh is accessed or
 referenced by some other class (e.g., MeshFieldLinear). Deforming the mesh can
 cause downstream dependencies to become invalid. Meshes should only be deformed
 by those who own all downstream dependencies and update them accordingly.

 This class cannot be moved or copied. It is assumed upon creation that it will
 be permanently associated with a *specific* MeshType instance and maintain that
 association for its entire lifetime (see DeformableVolumeMesh as an example).
 */
template <typename MeshType>
class MeshDeformer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshDeformer);

  /* The scalar type the mesh uses to report vertex positions. */
  using T = typename MeshType::ScalarType;

  /* Constructs a deformer for a particular `mesh_M`. `mesh_M` must remain alive
   for at least as long as this deformer. The mesh has its vertex positions
   measured and expressed in frame M; when updating the positions, new vertex
   position values must likewise be measured and expressed in this same frame.

   @param mesh_M  The mesh to be deformed.
   @pre `mesh_M != nullptr`. */
  explicit MeshDeformer(MeshType* mesh_M) : mesh_(*mesh_M) {
    DRAKE_DEMAND(mesh_M != nullptr);
  }

  /* Getter for the *const* version of the mesh to be deformed. */
  const MeshType& mesh() const { return mesh_; }

  /* Updates the position of all vertices in the mesh. Each sequential triple in
   p_MVs (e.g., 3i, 3i + 1, 3i + 2), i ∈ ℤ, is interpreted as a position vector
   associated with the iᵗʰ vertex. The position values are interpreted to be
   measured and expressed in the same frame as the mesh to be deformed.

   @param p_MVs  Vertex positions for the mesh's N vertices flattened into a
                 vector (where each position vector is measured and expressed in
                 the mesh's original frame).
   @throws std::exception if p_MVs.size() != 3 * mesh().num_vertices() */
  void SetAllPositions(const Eigen::Ref<const VectorX<T>>& p_MVs);

 private:
  MeshType& mesh_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
