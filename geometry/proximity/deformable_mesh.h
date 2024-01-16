#pragma once

#include <utility>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/bvh_updater.h"
#include "drake/geometry/proximity/mesh_deformer.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): When SceneGraph properly supports deformable meshes
//  in a public manner, move this out of internal and into geometry.
// TODO(xuchenhan-tri): Consider supporting a version of DeformableMesh that
//  doesn't have bvh for applications that don't need it.
/* Representation of a mesh whose vertices can be moved.

 This is currently a speculative feature and not formally supported by
 SceneGraph. When SceneGraph formally supports deformable geometry, this will
 come out of the `internal` namespace. */
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
      : mesh_(std::move(mesh_M)),
        deformer_(&mesh_),
        bvh_(mesh_),
        bvh_updater_(&mesh_, &bvh_) {}

  /* @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{

  // Note: Neither MeshDeformer nor BvhUpdater have copy/move semantics. So,
  // we can't default those semantics on DeformableMesh. The deformer_
  // and bvh_updater_ are configured to *always* point to the instance's
  // *members* mesh_ and bvh_. That never changes during the entire lifetime
  // of a DeformableMesh instance. So, the assignment operators only have to
  // worry about setting the member mesh and bvh to the assigned data; we
  // don't have to (and can't) make any modifications to the deformer or bvh
  // updater.

  DeformableMesh(const DeformableMesh& other)
      : DeformableMesh(other.mesh_, other.bvh_) {}

  DeformableMesh& operator=(const DeformableMesh& other) {
    if (this == &other) return *this;
    mesh_ = other.mesh();
    bvh_ = other.bvh_;
    return *this;
  }

  DeformableMesh(DeformableMesh&& other)
      : DeformableMesh(std::move(other.mesh_), std::move(other.bvh_)) {}

  DeformableMesh& operator=(DeformableMesh&& other) {
    if (this == &other) return *this;
    mesh_ = std::move(other.mesh_);
    bvh_ = std::move(other.bvh_);
    return *this;
  }

  //@}

  /* Access to the underlying mesh. Represent the most recent configuration
   based either on construction or a call to UpdateVertexPositions(). */
  const MeshType& mesh() const { return mesh_; }

  // TODO(SeanCurtis-TRI): We're currently returning an object defined in the
  //  internal namespace. Right now it's not bad because DeformableMesh is
  //  also defined in the internal namespace. But when *this* class moves into
  //  the geometry namespace, we'll either have to bring Bvh along or make
  //  this method private with friend access.
  /* The dynamic bounding volume hierarchy for this mesh. */
  const internal::Bvh<Aabb, MeshType>& bvh() const { return bvh_; }

  /* Updates the vertex positions of the underlying mesh.

  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size == 3 * mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<T>>& q);

 private:
  // The delegate constructor used by move and copy constructors. We can't
  // have all three constructors delegate to this same constructor because the
  // base constructor *creates* a bvh on the fly -- so, it is, in a sense,
  // optional. I have not been able to create a constructor that supports both
  // optional and maintains semantics. So, the mesh-only constructor defines
  // its own initialization.
  DeformableMesh(MeshType mesh_M, Bvh<Aabb, MeshType> bvh_M)
      : mesh_(std::move(mesh_M)),
        deformer_(&mesh_),
        bvh_(std::move(bvh_M)),
        bvh_updater_(&mesh_, &bvh_) {}

  MeshType mesh_;
  MeshDeformer<MeshType> deformer_;
  Bvh<Aabb, MeshType> bvh_;
  BvhUpdater<MeshType> bvh_updater_;
};

template <typename T>
using DeformableVolumeMesh = DeformableMesh<VolumeMesh<T>>;
template <typename T>
using DeformableTriangleSurfaceMesh = DeformableMesh<TriangleSurfaceMesh<T>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
