#pragma once

#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/surface_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <class T>
class MultibodyPlant;

namespace hydroelastic_contact {

/**
 * A class for computing the spatial forces on rigid bodies in a MultibodyPlant
 * as a function of the hydroelastic contact model.
 */
template <class T>
class HydroelasticTraction {
 public:
  explicit HydroelasticTraction(MultibodyPlant<T>* plant) : plant_(plant) {}

  /**
   Computes the traction (expressed in the world frame) at the point of contact
   `r_barycentric_M` (in barycentric coordinates of the given face of the given
   contact surface), given the dissipation and Coulomb friction coefficients.
   @param context the context of the MultibodyPlant.
   @param surface the contact surface computed by the hydroelastic contact
          model.
   @param face_index the index of the face in the contact surface.
   @param r_barycentric_M the barycentric coordinates giving the location of the
          point of contact, using the triangle of `contact_surface` specified by
          `face_index`.
   @param dissipation the non-negative dissipation coefficient that determines
          how rapidly energy is dissipated through the normal direction.
   @param mu_coulomb the non-negative Coulomb friction coefficient.
   @param[out] r_W on output, the offset vector from the origin of the world
               frame to the contact point, expressed in the world frame.
   @returns the traction at the contact point, expressed in the world frame.
   @pre `0 <= face_index < surface.mesh().num_faces()`
   @pre `r_barycentric_M.min() >= 0` and `r_barycentric_M.sum() = 1`.
   */
  Vector3<T> CalcTractionAtPoint(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric&
          r_barycentric_M, double dissipation, double mu_coulomb,
      Vector3<T>* r_W) const;

  /**
   Computes the spatial forces on the two bodies due to the traction at the
   given contact point.
   @param context the context of the MultibodyPlant.
   @param surface the contact surface computed by the hydroelastic contact
          model.
   @param r_W the offset vector from the origin of the world frame to the
          contact point, expressed in the world frame.
   @param traction_W the traction vector, expressed in the world frame, that
          points toward `surface.M_id()`.
   @param[out] f_Mo_W on return, the spatial force (due to the traction) that
               acts on the body attached to `surface.M_id()`.
   @param[out] f_No_W on return, the spatial force (due to the traction) that
               acts on the body attached to `surface.N_id()`.
   */
  void ComputeSpatialForcesAtBodyOriginsFromTraction(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      const Vector3<T>& r_W,
      const Vector3<T>& traction_W,
      multibody::SpatialForce<T>* f_Mo_W,
      multibody::SpatialForce<T>* f_No_W) const;

 private:
  const math::RigidTransform<T>& GetTransformFromGeometry(
      const systems::Context<T>& context,
      geometry::GeometryId geometry_id) const;
  Vector3<T> CalcContactPoint(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric& r_barycentric_M)
      const;

  MultibodyPlant<T>* plant_{nullptr};
};

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::hydroelastic_contact::HydroelasticTraction)
