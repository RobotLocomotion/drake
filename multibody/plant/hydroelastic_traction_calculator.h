#pragma once

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <class T>
class MultibodyPlant;

/**
 * A class for computing the spatial forces on rigid bodies in a MultibodyPlant
 * as a function of the hydroelastic contact model.
 */
template <class T>
class HydroelasticTractionCalculator {
 public:
  explicit HydroelasticTractionCalculator(
      MultibodyPlant<T>* plant) : plant_(plant) {}

  /**
   Computes the traction (expressed in the world frame) at the point of contact
   `Q_barycentric_M` (in barycentric coordinates of the given face of the given
   contact surface), given the dissipation and Coulomb friction coefficients.
   @param context the context of the MultibodyPlant.
   @param surface the contact surface computed by the hydroelastic contact
          model.
   @param face_index the index of the face in the contact surface.
   @param Q_barycentric_M the barycentric coordinates giving the location of the
          point of contact (Q), using the triangle of `contact_surface`
          specified by `face_index`.
   @param dissipation the non-negative dissipation coefficient that determines
          how rapidly energy is dissipated through the normal direction.
   @param mu_coulomb the non-negative Coulomb friction coefficient.
   @param[out] p_WQ on output, the offset vector from the origin of the world
               frame to the contact point Q, expressed in the world frame.
   @returns the traction at the contact point, expressed in the world frame.
   @pre `0 <= face_index < surface.mesh().num_faces()`
   @pre `Q_barycentric_M.min() >= 0` and `Q_barycentric_M.sum() = 1`.
   */
  Vector3<T> CalcTractionAtPoint(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric&
          Q_barycentric_M, double dissipation, double mu_coulomb,
      Vector3<T>* p_WQ) const;

  /**
   Computes the spatial forces on the two bodies due to the traction at the
   given contact point.
   @param context the context of the MultibodyPlant.
   @param surface the contact surface computed by the hydroelastic contact
          model.
   @param p_WQ the offset vector from the origin of the world frame to the
          contact point, expressed in the world frame.
   @param traction_Q_W the traction vector at Point Q, expressed in the world
          frame, that points toward `surface.M_id()`.
   @param[out] F_Mo_W on return, the spatial force (due to the traction) that
               acts on the body attached to `surface.M_id()`.
   @param[out] F_No_W on return, the spatial force (due to the traction) that
               acts on the body attached to `surface.N_id()`.
   */
  void ComputeSpatialForcesAtBodyOriginsFromTraction(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      const Vector3<T>& p_WQ,
      const Vector3<T>& traction_Q_W,
      multibody::SpatialForce<T>* F_Mo_W,
      multibody::SpatialForce<T>* F_No_W) const;

  /// Gets the regularization parameter used for friction (in m/s). The closer
  /// that this parameter is to zero, the closer that the regularized friction
  /// model will approximate Coulomb friction.
  double regularization_scalar() const { return vslip_regularizer_; }

 private:
  const math::RigidTransform<T> GetTransformFromGeometry(
      const systems::Context<T>& context,
      geometry::GeometryId geometry_id) const;
  Vector3<T> CalcContactPoint(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric& r_barycentric_M)
      const;

  MultibodyPlant<T>* plant_{nullptr};

  // The parameter for regularizing the Coulomb friction model.
  double vslip_regularizer_{1e-8};
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::HydroelasticTractionCalculator)
