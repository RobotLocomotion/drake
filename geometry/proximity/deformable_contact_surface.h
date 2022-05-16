#pragma once

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI) The application of the template parameter T is *not*
//  well reasoned. Currently, we're assuming that *all* quantities can and
//  should be expressed in the same scalar. It is *not* clear that *should*
//  be the case. But for now, we'll assume a homogenous scalar environment.

// TODO(xuchenhan-tri) Consider either beefing this struct up or creating some
//  other new data structure to report the actual contact mesh (instead of just
//  the contact data) for visualization purpose.
/* The per-polygon data associated with each contact polygon in the
 DeformableContactSurface between a deformable tetrahedral mesh and a rigid
 triangle mesh.*/
template <typename T>
struct ContactPolygonData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactPolygonData)
  /* The polygon's area. */
  T area;
  /* The unit vector normal to the polygon pointing out of the rigid triangle
   surface mesh, expressed in the *deformable* volume mesh's frame D. */
  Vector3<T> unit_normal;
  /* The polygon's centroid, measured and expressed in the *deformable*
   volume mesh's frame D. */
  Vector3<T> centroid;
  /* The polygon's centroid, described in barycentric coordinates of a
   tetrahedron drawn from the intersecting tet-mesh element. See `tet_index`. */
  Vector4<T> b_centroid;
  /* The index of the tetrahedron element in the intersecting tet-mesh in which
   this data's polygon is completely contained.  */
  int tet_index{};
};

/* Characterization of the contact surface between a deformable volume (tet)
 mesh and a rigid surface (tri) mesh. The contact surface is implicitly a
 polygon mesh, but only the per-polygon data (e.g, area, centroid and normal)
 can be queried (see ContactPolygonData).*/
template <typename T>
class DeformableContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactSurface)

  /* Constructs the contact surface from the given per-polygon data.
   Currently we do not store a polygonal mesh of the contact surface yet.
   If we do, we will assume poly_data[i] corresponds to the i-th polygon in
   the polygonal mesh. */
  explicit DeformableContactSurface(
      std::vector<ContactPolygonData<T>> poly_data)
      : polygon_data_(std::move(poly_data)) {}

  /* Returns `true` if this surface has no data -- there is no surface.  */
  bool empty() const { return polygon_data_.empty(); }

  /* Returns the number of polygons in the implicit polygon mesh. */
  int num_polygons() const { return polygon_data_.size(); }

  /* Returns the data associated with the polygon indexed by `poly_index`.
   @pre 0 <= poly_index < num_polygons(). */
  const ContactPolygonData<T>& polygon_data(int poly_index) const {
    return polygon_data_[poly_index];
  }

 private:
  /* The per-polygon data for the contact surface. */
  std::vector<ContactPolygonData<T>> polygon_data_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
