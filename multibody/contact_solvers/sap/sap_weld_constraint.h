#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using drake::math::RigidTransform;

/* Implements a SAP (compliant) weld constraint between two bodies.

 Consider a frame P on an object A and a frame Q on an object B. We define the
 intermediate frame N as the frame halfway between frame P and Q using linear
 interpolation for the pose of the origin of N and spherical linear
 interpolation for the rotational component of N's pose. Using poses expressed
 relative to the world frame:
   p_WNo = (p_WPo + p_WQo) / 2
   R_WN = slerp(R_WP, R_WQ, 0.5)
 We use this intermediate frame N so that computed impulses are
 symmetric wrt object A and object B. This constraint is modeled with 6
 constraint equations, which can be decomposed into a rotational component (3
 equations) and a translational component (3 equations).

 Let a = a_PQ_N = sinθ⋅k ≈ θ⋅k be a small angle approximation to the Euler
 vector representation of the relative rotation R_PQ, Q's orientation in P
 expressed in frame N. Let p = p_PoQo_N, Qo's position relative to Po expressed
 in frame N. This constraint penalizes the full constraint function, g, to
 impose:
   g = (a_PQ_N, p_PoQo_N) = 0 ∈ ℝ⁶
 with corresponding constraint velocity:
   ġ = vc(v) ≈ (w_PQ_N, v_PoQo_N) = V_N_PQ_N

 N.B. Only when P and Q are coincident is their relative velocity independent
 of which frame it is measured in. This means that the way we model this
 constraint lacks frame-invariance. We make an approximation measuring the
 relative velocity in the world frame for convenience, V_W_PQ ≈ V_N_PQ, leading
 to a small error of order O(‖p_PQ‖⋅‖w_WN‖). Non-coincident P and Q also leads
 to a small artifact discussed below where angular momentum is not conserved.
 See MakeSapHolonomicConstraintKinematics() for a derivation of this constraint
 velocity and discussion of these approximations.

 This leads to a constraint impulse γ, which is likewise composed of a
 rotational and translational component:
   γ = (γᵣ, γₜ) ∈ ℝ⁶
 This constraint impulse corresponds to a spatial impulse on B, applied at Q
 expressed in frame N:
   Γ_Bq_N = (γᵣ, γₜ)
 and a spatial impulse on A applied at P expressed in frame N:
   Γ_Ap_N = -(γᵣ, γₜ)

 N.B. See DoAccumulateSpatialImpulses() for a discussion of where this
 interpretation of γ comes from. In general when P and Q are not coincident this
 formulation does NOT conserve angular momentum, introducing a small moment of
 order O(‖γₜ‖⋅‖p_PQ‖).

 DoAccumulateSpatialImpulses() shifts these impulses to the body frame origins
 and re-expresses them in the world frame, reporting Γ_BBo_W and Γ_AAo_W,
 respectively.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapWeldConstraint final : public SapHolonomicConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Private copy construction is enabled for use in DoClone(). */
  //@{
  SapWeldConstraint& operator=(const SapWeldConstraint&) = delete;
  SapWeldConstraint(SapWeldConstraint&&) = delete;
  SapWeldConstraint& operator=(SapWeldConstraint&&) = delete;
  //@}

  /* Class to store the kinematics of the the constraint in its current
   configuration, when it gets constructed. See SapWeldConstraint class
   documentation for definitions of frame and quantites used here. */
  class Kinematics {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Kinematics);

    /* @param[in] objectA
         Index of the physical object A on which frame P attaches.
       @param[in] X_WP Pose of frame P in the world frame.
       @param[in] p_AP_W Position of point P in A, expressed in the world frame.
       @param[in] objectB
         Index of the physical object B on which frame Q attaches.
       @param[in] X_WQ Pose of frame Q in the world frame.
       @param[in] p_BQ_W Position of point Q in B, expressed in the world frame.
       @param[in] J_W_PQ Jacobian for the relative spatial velocity V_W_PQ.

       @note Thus far only dense Jacobian blocks are supported, i.e. rigid body
       applications in mind.
       @throws std::exception If J_W_PQ.blocks_are_dense() == false.
    */
    Kinematics(int objectA, RigidTransform<T> X_WP, Vector3<T> p_AP_W,
               int objectB, RigidTransform<T> X_WQ, Vector3<T> p_BQ_W,
               SapConstraintJacobian<T> J_W_PQ);

    int objectA() const { return objectA_; }
    const RigidTransform<T>& X_WP() const { return X_WP_; }
    const Vector3<T>& p_AP_W() const { return p_AP_W_; }
    int objectB() const { return objectB_; }
    const RigidTransform<T>& X_WQ() const { return X_WQ_; }
    const Vector3<T>& p_BQ_W() const { return p_BQ_W_; }
    const SapConstraintJacobian<T>& jacobian() const { return J_; }

    const RigidTransform<T>& X_WN() const { return X_WN_; }
    const Vector3<T>& p_PoQo_N() const { return p_PoQo_N_; }
    const Vector3<T>& a_PQ_N() const { return a_PQ_N_; }

   private:
    /* Index to a physical object A. */
    int objectA_;

    /* Pose of frame P in the world frame. */
    RigidTransform<T> X_WP_;

    /* Position of point P in A, expressed in the world frame. */
    Vector3<T> p_AP_W_;

    /* Index to a physical object B. */
    int objectB_;

    /* Pose of frame Q in the world. */
    RigidTransform<T> X_WQ_;

    /* Position of point Q in B, expressed in the world frame.  */
    Vector3<T> p_BQ_W_;

    /* Jacobian that defines the spatial velocity V_W_PQ of frame P relative
     to point Q, expressed in the world frame. That is, V_W_PQ = J⋅v. */
    SapConstraintJacobian<T> J_;

    /* Pose of intermediate frame N in the world frame. */
    RigidTransform<T> X_WN_;

    /* Position vector from Po to point Qo expressed in frame N. */
    Vector3<T> p_PoQo_N_;

    /* Rotation relating frame P to frame Q expressed as the axis angle
     * product a = θ⋅k expressed in frame N */
    Vector3<T> a_PQ_N_;
  };

  /* Constructs a weld constraint given its kinematics in a particular
   configuration */
  explicit SapWeldConstraint(Kinematics kinematics);

  const Kinematics& kinematics() const { return kinematics_; }

 private:
  /* Private copy construction is enabled to use in the implementation of
     DoClone(). */
  SapWeldConstraint(const SapWeldConstraint&) = default;

  /* no-op for this constraint. */
  void DoAccumulateGeneralizedImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                       EigenPtr<VectorX<T>>) const final {}

  /* Accumulates spatial impulses applied by this constraint on the i-th
   object.
   @param[in] i Object index. As defined at construction i = 0 corresponds to
   object A and i = 1 corresponds to object B.
   @param[in] gamma A vector of size 6, with the constraint impulse.
   @param[out] F On output, this method accumulates the total spatial impulse
   applied by this constraint on the i-th object.
   @pre 0 ≤ i < 2.
   @pre F is not nullptr. */
  void DoAccumulateSpatialImpulses(int i,
                                   const Eigen::Ref<const VectorX<T>>& gamma,
                                   SpatialForce<T>* F) const final;

  /* @returns Holonomic constraint parameters needed at construction of the
    parent SapHolonomicConstraint. */
  static typename SapHolonomicConstraint<T>::Parameters
  MakeSapHolonomicConstraintParameters();

  /* @returns Holonomic constraint kinematics needed at construction of the
    parent SapHolonomicConstraint. */
  static typename SapHolonomicConstraint<T>::Kinematics
  MakeSapHolonomicConstraintKinematics(const Kinematics& kinematics);

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapWeldConstraint<T>>(
        new SapWeldConstraint<T>(*this));
  }

  Kinematics kinematics_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
