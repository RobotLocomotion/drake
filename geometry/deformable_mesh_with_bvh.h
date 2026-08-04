#pragma once

#include <utility>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/bvh_updater.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): When SceneGraph properly supports deformable meshes
//  in a public manner, move this out of internal and into geometry.
/* Representation of a mesh whose vertices can be moved. The mesh is equipped
 with a bounding volume hierarchy (BVH) acceleration structure that is updated
 every time the vertices are moved.

 @tparam MeshType TriangleSurfaceMesh<T> or VolumeMesh<T> where T is double or
                  AutoDiffXd. */
template <typename MeshType>
class DeformableMeshWithBvh {
 public:
  using T = typename MeshType::ScalarType;

  /* Constructs a deformable representation of the given mesh whose vertex
   positions are measured and expressed in Frame M. When the vertex positions
   are updated, those values must likewise be measured and expressed in this
   same frame.

   The %DeformableMeshWithBvh will *own* its own underlying mesh and constructs
   a BVH on that mesh. */
  explicit DeformableMeshWithBvh(MeshType mesh_M)
      : deformable_mesh_(mesh_M), bvh_(mesh()), bvh_updater_(&mesh(), &bvh_) {}

  /* @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{

  // Note: BvhUpdater doesn't have copy/move semantics. So, we can't default
  // those semantics on DeformableMeshWithBvh. The bvh_updater_ is configured to
  // *always* point to the instance's *members* mesh and bvh. That never changes
  // during the entire lifetime of a DeformableMeshWithBvh instance. So, the
  // assignment operators only have to worry about setting the member mesh and
  // bvh to the assigned data; we don't have to (and can't) make any
  // modifications to the bvh updater.

  DeformableMeshWithBvh(const DeformableMeshWithBvh& other)
      : DeformableMeshWithBvh(other.deformable_mesh_, other.bvh_) {}

  DeformableMeshWithBvh& operator=(const DeformableMeshWithBvh& other) {
    if (this == &other) return *this;
    // Update the mesh and bvh in place, allowing the pointer values in
    // bvh_updater_ and deformer_ to remain valid.
    deformable_mesh_ = other.deformable_mesh_;
    bvh_ = other.bvh_;
    return *this;
  }

  DeformableMeshWithBvh(DeformableMeshWithBvh&& other)
      : DeformableMeshWithBvh(std::move(other.deformable_mesh_),
                              std::move(other.bvh_)) {}

  DeformableMeshWithBvh& operator=(DeformableMeshWithBvh&& other) {
    if (this == &other) return *this;
    deformable_mesh_ = std::move(other.deformable_mesh_);
    bvh_ = std::move(other.bvh_);
    return *this;
  }

  //@}

  /* Access to the underlying mesh. Represents the most recent configuration
   based either on construction or a call to UpdateVertexPositions(). */
  const MeshType& mesh() const { return deformable_mesh_; }

  // TODO(SeanCurtis-TRI): We're currently returning an object defined in the
  //  internal namespace. Right now it's not bad because DeformableMeshWithBvh
  //  is also defined in the internal namespace. But when *this* class moves
  //  into the geometry namespace, we'll either have to bring Bvh along or make
  //  this method private with friend access.
  /* The dynamic bounding volume hierarchy for this mesh. */
  const internal::Bvh<Aabb, MeshType>& bvh() const { return bvh_; }

  /* Updates the vertex positions of the underlying mesh as well as the bounding
   volume hierarchy of the mesh.

  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size == 3 * mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<T>>& q);

 private:
  // The delegate constructor used by move and copy constructors. The mesh-only
  // constructor cannot delegate to this function because it would have to both
  // construct a BVH from its input mesh *and* move that mesh. The C++
  // specification leaves the ordering of evaluating those two expressions as
  // unspecified. We cannot guarantee that we'll construct the BVH before moving
  // the mesh's contents out. So, it has its own initialization.
  DeformableMeshWithBvh(MeshType deformable_mesh_M, Bvh<Aabb, MeshType> bvh_M)
      : deformable_mesh_(std::move(deformable_mesh_M)),
        bvh_(std::move(bvh_M)),
        bvh_updater_(&mesh(), &bvh_) {}

  MeshType deformable_mesh_;
  Bvh<Aabb, MeshType> bvh_;
  BvhUpdater<MeshType> bvh_updater_;
};

template <typename T>
using DeformableVolumeMeshWithBvh = DeformableMeshWithBvh<VolumeMesh<T>>;
template <typename T>
using DeformableSurfaceMeshWithBvh =
    DeformableMeshWithBvh<TriangleSurfaceMesh<T>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
