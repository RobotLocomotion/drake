#pragma once

#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_volume_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace fem {

// TODO(SeanCurtis-TRI) The application of the template parameter T is *not*
//  well reasoned. Currently, we're assuming that *all* quantities can and
//  should be expressed in the same scalar. It is *not* clear that *should*
//  be the case. But for now, we'll assume a homogenous scalar environment.

// TODO(xuchenhan-tri) Consider either beefing this struct upon or creating some
//  other new data structure to report the actual contact mesh (instead of just
//  the contact data) for visualization purpose.
/** The per-polygon data associated with each contact polygon in the
 DeformableContactSurface between a deformable tetrahedral mesh and a rigid
 triangle mesh.*/
template <typename T>
struct ContactPolygonData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactPolygonData)
  /** The polygon's area. */
  T area;
  /** The unit vector normal to the polygon pointing out of the rigid triangle
   surface mesh, expressed in the *deformable* volume mesh's frame D. */
  Vector3<T> unit_normal;
  /** The polygon's centroid, measured and expressed in the *deformable*
   volume mesh's frame D. */
  Vector3<T> centroid;
  /** The polygon's centroid, described in barycentric coordinates of a
   tetrahedron drawn from the intersecting tet-mesh element. See `tet_index`. */
  Vector4<T> b_centroid;
  /** The index of the tetrahedron element in the intersecting tet-mesh in which
   this data's polygon is completely contained.  */
  int tet_index{};
};

/** Characterization of the contact surface between a deformable volume (tet)
 mesh and a rigid surface (tri) mesh. The contact surface is implicitly a
 polygon mesh, but only the per-polygon data (e.g, area, centroid and normal)
 can be queried (see ContactPolygonData).*/
template <typename T>
class DeformableContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactSurface)

  /** Constructs the contact surface from the given per-polygon data. The
   corresponding implicit polygon mesh is indexed from 0 to
   poly_data.size()-1. */
  explicit DeformableContactSurface(
      std::vector<ContactPolygonData<T>> poly_data)
      : polygon_data_(std::move(poly_data)) {}

  /** Returns `true` if this surface has no data -- there is no surface.  */
  bool empty() const { return polygon_data_.empty(); }

  /** Returns the number of polygons in the implicit polygon mesh. */
  int num_polygons() const { return polygon_data_.size(); }

  /** Returns the data associated with the polygon indexed by `poly_index`. */
  const ContactPolygonData<T>& polygon_data(int poly_index) const {
    return polygon_data_[poly_index];
  }

 private:
  /* The per-polygon data for the contact surface. */
  std::vector<ContactPolygonData<T>> polygon_data_;
};

// TODO(SeanCurtis-TRI) This needs some acceleration; we need an OBB BVH for
//  the triangle mesh and an AABB BVH for the tet mesh (one that updates
//  based on deformations). The Obb BVH exists and we could use it assuming
//  we add the OBB-Tet intersection test.

/** Computes the contact between a deformable tet mesh and a rigid tri mesh. The
 contact is characterized by a collection of per-polygon quantities (see
 DeformableContactSurface) associated with an implicit underlying polygon mesh.

 If there is no contact, the result will be empty.

 @param tet_mesh_D   The deformable tetrahedral mesh, with vertex positions
                     measured and expressed in the *deformable* mesh's frame D.
 @param tri_mesh_R   The triangle mesh, with vertex positions measured and
                     expressed in the *rigid* mesh's frame R.
 @param bvh_R        The bounding volume hierarchy for the triangle mesh,
                     measured and expressed in the *rigid* mesh's frame R.
 @param X_DR         The pose of the triangle mesh in the volume mesh's frame.
 @returns The collection of contact data associated with an implicit contact
          surface formed by the intersection of the volume and surface meshes.
          If there is no intersection, the resulting surface will report as
          "empty".  */
template <typename T>
DeformableContactSurface<T> ComputeTetMeshTriMeshContact(
    const geometry::internal::DeformableVolumeMesh<T>& tet_mesh_D,
    const geometry::TriangleSurfaceMesh<double>& tri_mesh_R,
    const geometry::internal::Bvh<geometry::internal::Obb,
                                  geometry::TriangleSurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<T>& X_DR);
}  // namespace fem
}  // namespace multibody
}  // namespace drake
