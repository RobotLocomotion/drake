#pragma once

#include <utility>

#include "drake/geometry/proximity/mesh_deformer.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): When SceneGraph properly supports deformable meshes
//  in a public manner, move this out of internal and into geometry.
/* Representation of a mesh whose vertices can be moved. Consider using
 DeformableMeshWithBvh instead if you need a bounding volume hierarchy (BVH)
 acceleration structure for proximity queries.

 @tparam MeshType TriangleSurfaceMesh<T> or VolumeMesh<T> where T is double or
                  AutoDiffXd. */
template <typename MeshType>
class DeformableMesh {
 public:
  using T = typename MeshType::ScalarType;

  /* Constructs a deformable representation of the given mesh whose vertex
   positions are measured and expressed in Frame M. When the vertex positions
   are updated, those values must likewise be measured and expressed in this
   same frame.

   The %DeformableMesh will *own* its own underlying mesh. */
  explicit DeformableMesh(MeshType mesh_M)
      : mesh_(std::move(mesh_M)), deformer_(&mesh_) {}

  /* @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{

  // Note: MeshDeformer doesn't have copy/move semantics. So, we can't default
  // those semantics on DeformableMesh. The deformer_ is configured to *always*
  // point to the instance's *members* mesh_. That never changes during the
  // entire lifetime of a DeformableMesh instance. So, the assignment operators
  // only have to worry about setting the member mesh to the assigned data; we
  // don't have to (and can't) make any modifications to the deformer.

  DeformableMesh(const DeformableMesh& other) : DeformableMesh(other.mesh_) {}

  DeformableMesh& operator=(const DeformableMesh& other) {
    if (this == &other) return *this;
    // Update the mesh in place, allowing the pointer values in and deformer_ to
    // remain valid.
    mesh_ = other.mesh();
    return *this;
  }

  DeformableMesh(DeformableMesh&& other)
      : DeformableMesh(std::move(other.mesh_)) {}

  DeformableMesh& operator=(DeformableMesh&& other) {
    if (this == &other) return *this;
    mesh_ = std::move(other.mesh_);
    return *this;
  }

  //@}

  /* Access to the underlying mesh. Represent the most recent configuration
   based either on construction or a call to UpdateVertexPositions(). */
  const MeshType& mesh() const { return mesh_; }

  /* Updates the vertex positions of the underlying mesh.

  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size == 3 * mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<T>>& q);

 private:
  MeshType mesh_;
  MeshDeformer<MeshType> deformer_;
};

template <typename T>
using DeformableTriangleMesh = DeformableMesh<TriangleSurfaceMesh<T>>;
template <typename T>
using DeformableVolumeMesh = DeformableMesh<VolumeMesh<T>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
