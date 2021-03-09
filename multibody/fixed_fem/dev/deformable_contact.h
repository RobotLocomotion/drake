#pragma once

#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

// TODO(SeanCurtis-TRI) The application of the template parameter T is *not*
//  well reasoned. Currently, we're assuming that *all* quantities can and
//  should be expressed in the same scalar. It is *not* clear that *should*
//  be the case. But for now, we'll assume a homogenous scalar environment.

/** The per-triangle data associated with the mesh triangles in a
 DeformableContactSurface that *cannot* be queried from the contact surface's
 mesh.  */
template <typename T>
struct ContactTriangleData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactTriangleData)
  /** The triangle's centroid, measured and expressed in the the *deformable*
   volume mesh's frame D. */
  Vector3<T> centroid;
  /** The triangle's centroid, described in barycentric coordinates of a
   tetrahedron drawn from the intersecting tet-mesh element. See `tet_index`. */
  Vector4<T> b_centroid;
  /** The index of the tetrahedron element in the intersecting tet-mesh in which
   this data's triangle is completely contained.  */
  geometry::VolumeElementIndex tet_index;
};

/** Characterization of the contact surface between a deformable volume (tet)
 mesh and a rigid surface (tri) mesh. The contact surface is, itself, a triangle
 mesh. The mesh can be directly queried for per-triangle data (e.g., area and
 normal). This class also contains additional per-triangle data (see
 ContactTriangleData).  */
template <typename T>
class DeformableContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactSurface)

  /** Constructs the contact surface from the given mesh (with vertex positions
   measured and expressed in the *deformable* volume mesh's frame D) and
   per-triangle data.
   @pre tri_data.size() == mesh_D.num_elements().  */
  DeformableContactSurface(geometry::SurfaceMesh<T> mesh_D,
                           std::vector<ContactTriangleData<T>> tri_data)
      : mesh_D_(std::move(mesh_D)), triangle_data_(std::move(tri_data)) {
    DRAKE_DEMAND(mesh_D_.num_elements() ==
                 static_cast<int>(triangle_data_.size()));
  }

  /** Returns `true` if this surface has no data -- there is no surface.  */
  bool is_empty() const { return mesh_D_.num_elements() == 0; }

  /** Returns the contact surface's underlying triangle mesh with vertices
   measured and expressed in the *deformable* volume mesh's frame D.  */
  const geometry::SurfaceMesh<T>& mesh() const { return mesh_D_; }

  /** Returns the data associated with the triangle indexed by `tri_index` in
   mesh().  */
  const ContactTriangleData<T>& triangle_data(int tri_index) const {
    return triangle_data_[tri_index];
  }

 private:
  /* The surface mesh spanning the contact surface. Vertex positions are
  measured and expressed in the *deformable* volume mesh's frame D.  */
  geometry::SurfaceMesh<T> mesh_D_;

  /* The per-triangle data for the contact surface. It should be an invariant
   that mesh_D_.num_elements() == triangle_data_.size().  */
  std::vector<ContactTriangleData<T>> triangle_data_;
};

// TODO(SeanCurtis-TRI) This needs some acceleration; we need an OBB BVH for
//  the triangle mesh and an AABB BVH for the tet mesh (one that updates
//  based on deformations). The Obb BVH exists and we could use it assuming
//  we add the OBB-Tet intersection test.

/** Computes the contact between a deformable tet mesh and a rigid tri mesh. The
 contact is characterized by contact surface consisting of a triangle mesh and
 a collection of additional per-triangle quantities (see
 DeformableContactSurface).

If there is no contact, the result will be empty.

@param tet_mesh_D   The tetrahedral mesh, with vertex positions measured and
                    expressed in the *deformable* mesh's frame D.
@param tri_mesh_R   The triangle mesh, with vertex positions measured and
                    expressed in the *rigid* mesh's frame R.
@param X_DR         The pose of the triangle mesh in the volume mesh's frame.
@returns The contact surface formed by the intersection of the volume and
         surface meshes. If there is no intersection, the resulting surface will
         report as "empty".  */
template <typename T>
DeformableContactSurface<T> ComputeTetMeshTriMeshContact(
    const geometry::VolumeMesh<T>& tet_mesh_D,
    const geometry::SurfaceMesh<double>& tri_mesh_R,
    const math::RigidTransform<T>& X_DR);
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
