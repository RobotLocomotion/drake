#pragma once

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <class T>
class MultibodyPlant;

namespace internal {

/**
 A class for computing the spatial forces on rigid bodies in a MultibodyPlant
 using the hydroelastic contact model, as described in:

 [Elandt, 2019]  R. Elandt, E. Drumwright, M. Sherman, and A. Ruina.
 A pressure field model for fast, robust approximation of net contact force and
 moment between nominally rigid objects. Proc. IEEE/RSJ Intl. Conf. on
 Intelligent Robots and Systems (IROS), 2019.
 */
template <typename T>
class HydroelasticTractionCalculator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HydroelasticTractionCalculator)

  HydroelasticTractionCalculator() {}

  /**
   Gets the regularization parameter used for friction (in m/s). The closer
   that this parameter is to zero, the closer that the regularized friction
   model will approximate Coulomb friction.
   */
  double regularization_scalar() const { return vslip_regularizer_; }

  /**
   Applies the hydroelastic model to two geometries defined in `surface`,
   resulting in a pair of spatial forces at the origins of two body frames.
   The body frames, A and B, are those to which `surface.M_id()` and
   `surface.N_id()` are affixed, respectively.
   */
  void ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
       const systems::Context<T>& context,
       const MultibodyPlant<T>& plant,
       const geometry::ContactSurface<T>& surface,
       double dissipation, double mu_coulomb,
       multibody::SpatialForce<T>* F_Ao_W,
       multibody::SpatialForce<T>* F_Bo_W) const;

 private:
  // To allow GTEST to test private functions.
  friend class MultibodyPlantHydroelasticTractionTests;

  // Data structure for storing quantities used repeatedly in the hydroelastic
  // traction calculations.
  class HydroelasticTractionCalculatorData {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticTractionCalculatorData)

    // @param context the MultibodyPlant context.
    // @param plant the plant used to compute the data for the
    //        traction calculations.
    // @param surface a pointer to the contact surface computed by the
    //        hydroelastic contact model that will be maintained for the life of
    //        this object.
    HydroelasticTractionCalculatorData(
        const systems::Context<T>& context,
        const MultibodyPlant<T>& plant,
        const geometry::ContactSurface<T>* surface);

    // Gets the ContactSurface passed to the data structure on construction.
    const geometry::ContactSurface<T>& surface() const { return surface_; }

    // Gets the surface centroid C, measured and expressed in World.
    const Vector3<T>& p_WC() const { return p_WC_; }

    // Gets the pose from Body A (the body that Geometry `surface.M_id()` in the
    // contact surface is affixed to) relative to the world frame.
    const math::RigidTransform<T>& X_WA() const { return X_WA_; }

    // Gets the pose from Body B (the body that Geometry `surface.N_id()` in the
    // contact surface is affixed to) relative to the world frame.
    const math::RigidTransform<T>& X_WB() const { return X_WB_; }

    // Gets the pose from Geometry `surface.M_id()` in the contact surface to
    // the world frame.
    const math::RigidTransform<T>& X_WM() const { return X_WM_; }

    // Gets the spatial velocity of Body A (the body that Geometry
    // `surface.M_id()` in the contact surface is affixed to) at the origin of
    // A's frame, measured in the world frame and expressed in the world frame.
    const SpatialVelocity<T>& V_WA() const { return V_WA_; }

    // Gets the spatial velocity of Body B (the body that Geometry
    // `surface.N_id()` in the contact surface is affixed to) at the origin of
    // B's frame, measured in the world frame and expressed in the world frame.
    const SpatialVelocity<T>& V_WB() const { return V_WB_; }

   private:
    const geometry::ContactSurface<T>& surface_;
    Vector3<T> p_WC_;
    math::RigidTransform<T> X_WM_;
    math::RigidTransform<T> X_WA_;
    math::RigidTransform<T> X_WB_;
    SpatialVelocity<T> V_WA_;
    SpatialVelocity<T> V_WB_;
  };

  Vector3<T> CalcTractionAtPoint(
      const HydroelasticTractionCalculatorData& data,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric& Q_barycentric,
      double dissipation, double mu_coulomb, Vector3<T>* p_WQ) const;

  multibody::SpatialForce<T> ComputeSpatialTractionAtAcFromTractionAtAq(
      const HydroelasticTractionCalculatorData& data, const Vector3<T>& p_WQ,
      const Vector3<T>& traction_Aq_W) const;

  // The parameter (in m/s) for regularizing the Coulomb friction model.
  double vslip_regularizer_{1e-6};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

// TODO(edrumwri) instantiate on SymbolicExpression when it no longer
// causes a linker error complaining about an unresolved symbol in SceneGraph.
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::internal::HydroelasticTractionCalculator)
