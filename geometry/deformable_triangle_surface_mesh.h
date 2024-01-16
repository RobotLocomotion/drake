#pragma once

#include <utility>

#include "drake/geometry/proximity/mesh_deformer.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* A triangle surface mesh whose vertex positions can change. When those vertex
 positions change, related quantities such as the face normals and centroids
 change accordingly with them. */
class DeformableTriangleSurfaceMesh {
 public:
  /* Constructs a deformable representation of the given mesh whose vertex
   positions are measured and expressed in Frame M. When the vertex positions
   are updated, those values must likewise be measured and expressed in this
   same frame.

   The %DeformableTriangleSurfaceMesh will *own* its own underlying
   TriangleSurfaceMesh. */
  DeformableTriangleSurfaceMesh(TriangleSurfaceMesh<double> mesh)
      : mesh_(std::move(mesh)), deformer_(&mesh_) {}

  /* @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{
  DeformableTriangleSurfaceMesh(const DeformableTriangleSurfaceMesh& other);

  DeformableTriangleSurfaceMesh& operator=(
      const DeformableTriangleSurfaceMesh& other);

  DeformableTriangleSurfaceMesh(DeformableTriangleSurfaceMesh&& other);

  DeformableTriangleSurfaceMesh& operator=(
      DeformableTriangleSurfaceMesh&& other);
  //@}

  /* Access to the underlying mesh. Represent the most recent configuration
   based either on construction or a call to UpdateVertexPositions(). */
  const TriangleSurfaceMesh<double>& mesh() const { return mesh_; }

  /* Updates the vertex positions of the underlying mesh.

  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size == 3 * mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<double>>& q) {
    deformer_.SetAllPositions(q);
  }

 private:
  TriangleSurfaceMesh<double> mesh_;
  MeshDeformer<TriangleSurfaceMesh<double>> deformer_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
