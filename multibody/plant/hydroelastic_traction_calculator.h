#pragma once

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {

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
  /// Data structure for storing quantities used repeatedly in the hydroelastic
  /// traction calculations. Documentation for parameter names can be found in
  /// the corresponding member documentation.
  struct HydroelasticTractionCalculatorData {
    HydroelasticTractionCalculatorData(
        const math::RigidTransform<T>& X_WA,
        const math::RigidTransform<T>& X_WB,
        const SpatialVelocity<T>& V_WA,
        const SpatialVelocity<T>& V_WB,
        const math::RigidTransform<T>& X_WM,
        const Vector3<T>& p_WC,
        geometry::ContactSurface<T> const* surface) :
            X_WA_(X_WA), X_WB_(X_WB), V_WA_(V_WA), V_WB_(V_WB),
            X_WM_(X_WM), p_WC_(p_WC), surface_(surface) { }

    // The pose of Body A (the body that Geometry `surface.M_id()` in the
    // contact surface is affixed to) in the world frame.
    math::RigidTransform<T> X_WA_;

    // The pose of Body B (the body that Geometry `surface.N_id()` in the
    // contact surface is affixed to) in the world frame.
    math::RigidTransform<T> X_WB_;

    // The spatial velocity of Body A (the body that Geometry
    // `surface.M_id()` in the contact surface is affixed to) at the origin of
    // A's frame, measured in the world frame and expressed in the world frame.
    SpatialVelocity<T> V_WA_;

    // The spatial velocity of Body B (the body that Geometry
    // `surface.N_id()` in the contact surface is affixed to) at the origin of
    // B's frame, measured in the world frame and expressed in the world frame.
    SpatialVelocity<T> V_WB_;

    // The pose of Geometry `surface.M_id()` in the world frame.
    math::RigidTransform<T> X_WM_;

    // The ContactSurface's centroid C, measured and expressed in World.
    Vector3<T> p_WC_;

    /// A pointer to the ContactSurface that will be maintained for the life
    /// of this object.
    geometry::ContactSurface<T> const* surface_;
  };

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
   @param data Relevant kinematic data.
   @param dissipation the nonnegative coefficient (in s/m) for dissipating
          energy along the direction of the surface normals.
   @param mu_coulomb the nonnegative coefficient for Coulomb friction.
   @param[output] F_Ao_W the spatial force on Body A, on return.
   @param[output] F_Bo_W the spatial force on Body B, on return.
   */ 
  void ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
       const HydroelasticTractionCalculatorData& data,
       double dissipation, double mu_coulomb,
       multibody::SpatialForce<T>* F_Ao_W,
       multibody::SpatialForce<T>* F_Bo_W) const;

 private:
  // To allow GTEST to test private functions.
  friend class MultibodyPlantHydroelasticTractionTests;

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
