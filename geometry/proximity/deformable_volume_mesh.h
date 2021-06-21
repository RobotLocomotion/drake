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
        deformer_(&mesh_),
        bvh_(mesh_),
        bvh_updater_(&mesh_, &bvh_) {}

  /* @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{

  // Note: Neither MeshDeformer nor BvhUpdater have copy/move semantics. So,
  // we can't default semantics on DeformableVolumeMesh. However, we rely on the
  // copy and move semantics of mesh and bvh to guarantee that the relationship
  // between an instance's mesh_ and deformer_ members is preserved, and
  // likewise between bvh_ and bvh_updater_. Therefore, in copying and moving
  // we make no direct changes to either deformer_ or bvh_updater_.

  // TODO(SeanCurtis-TRI): This incurs the cost of *building* the BVH anew
  //  instead of copying it. However, copying meshes is generally a bad idea. We
  //  assume that if copying happens at all, it'll only happen at
  //  initialization. Therefore, the cost of rebuilding (instead of copying)
  //  will disappear in the initialization. If it proves to be a problem, we
  //  can copy the Bvh and wire things together explicitly.
  DeformableVolumeMesh(const DeformableVolumeMesh& other)
      : DeformableVolumeMesh(other.mesh()) {}

  DeformableVolumeMesh& operator=(const DeformableVolumeMesh& other) {
    if (this == &other) return *this;
    mesh_ = other.mesh();
    // TODO(SeanCurtis-TRI): This likewise *builds* a new Bvh instead of
    //  copying. If this proves  to be problematic, swap it to a copy. Again,
    //  as part of initialization, it's probably not really a problem.
    bvh_ = Bvh<Aabb, VolumeMesh<T>>(mesh_);
    return *this;
  }

  DeformableVolumeMesh(DeformableVolumeMesh&& other)
      : mesh_(std::move(other.mesh_)),
        deformer_(&mesh_),
        bvh_(std::move(other.bvh_)),
        bvh_updater_(&mesh_, &bvh_) {}

  DeformableVolumeMesh& operator=(DeformableVolumeMesh&& other) {
    mesh_ = std::move(other.mesh_);
    bvh_ = std::move(other.bvh_);
    return *this;
  }

  //@}

  /* Access to the underlying mesh. Represent the most recent configuration
   based either on construction or a call to UpdateVertexPositions(). */
  const VolumeMesh<T>& mesh() const { return mesh_; }

  // TODO(SeanCurtis-TRI): We're currently returning an object defined in the
  //  internal namespace. Right now it's not bad because DeformableVolumeMesh is
  //  also defined in the internal namespace. But when *this* class moves into
  //  the geometry namespace, we'll either have to bring Bvh along or make this
  //  method private with friend access.
  /* The dynamic bounding volume hierarchy for this mesh. */
  const internal::Bvh<Aabb, VolumeMesh<T>>& bvh() const { return bvh_; }

  /* Updates the vertex positions of the underlying mesh.

  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size == 3 * mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<T>>& q);

 private:
  VolumeMesh<T> mesh_;
  MeshDeformer<VolumeMesh<T>> deformer_;
  Bvh<Aabb, VolumeMesh<T>> bvh_;
  BvhUpdater<VolumeMesh<T>> bvh_updater_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
