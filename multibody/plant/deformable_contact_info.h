#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/hydroelastic_quadrature_point_data.h"

namespace drake {
namespace multibody {

/** A class containing information regarding contact and contact response
 between two geometries belonging to a pair of bodies with at least one of them
 being a deformable body. This class provides:

    - The shared contact mesh between the two geometries.
    - The tractions acting at the contact points on the contact mesh.
    - The slip speeds at the contact points on the contact mesh.
    - The spatial force from the integrated tractions that is applied at the
      centroid of the contact surface.

 The two geometries are denoted as A and B respectively with geometry A
 guaranteed to be belonging to a deformable body.

 @experimental
 @tparam_default_scalar */
template <typename T>
class DeformableContactInfo {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactInfo);

  /** Constructs a %DeformableContactInfo.
   @param[in] id_A The geometry id of the deformable geometry A.
   @param[in] id_B The geometry id of geometry B.
   @param[in] contact_mesh_W  The contact mesh between geometries A and B in the
   world frame.
   @param[in] F_Ac_W Spatial force acting on body A, at contact mesh centroid C,
   and expressed in the world frame.
   @param[in] contact_point_data The vector of per-contact-point data where the
   contact points are sampled at the centroid of each face of the contact mesh.
   This vector is ordered the same way as the faces of `contact_mesh_W`.
   @pre id_A corresponds to a deformable geometry. */
  DeformableContactInfo(
      geometry::GeometryId id_A, geometry::GeometryId id_B,
      geometry::PolygonSurfaceMesh<T> contact_mesh_W, SpatialForce<T> F_Ac_W,
      std::vector<DeformableContactPointData<T>> contact_point_data);

  /** The geometry id of geometry A, guaranteed to belong to a deformable body
   in contact. */
  geometry::GeometryId id_A() const { return id_A_; }

  /** The geometry id of geometry B. */
  geometry::GeometryId id_B() const { return id_B_; }

  /** Returns a reference to the contact mesh expressed in the world frame. */
  const geometry::PolygonSurfaceMesh<T>& contact_mesh() const {
    return contact_mesh_W_;
  }

  /** Gets the spatial force applied on the deformable body associated with
   geometry A, at the centroid point C of the contact surface mesh, and
   expressed in the world frame W. */
  const SpatialForce<T>& F_Ac_W() const { return F_Ac_W_; }

  /** Gets the contact point data, including tractions and slip velocities, at
   each contact point located at the centroid of each element of the contact
   mesh. Ordered the same way as the faces of contact_mesh()`. */
  const std::vector<DeformableContactPointData<T>>& contact_point_data() const {
    return contact_point_data_;
  }

 private:
  /* The geometry ids of the geometries in contact. Geometry A is guaranteed to
   be deformable. */
  geometry::GeometryId id_A_;
  geometry::GeometryId id_B_;

  /* The mesh of the contact surface is defined in the world frame. */
  geometry::PolygonSurfaceMesh<T> contact_mesh_W_;

  /* The spatial force applied at the centroid (Point C) of the contact surface
   mesh. */
  SpatialForce<T> F_Ac_W_;

  /* Per-contact-point data evaluated at the centroid of each face of the
   contact surface mesh. */
  std::vector<DeformableContactPointData<T>> contact_point_data_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::DeformableContactInfo)
