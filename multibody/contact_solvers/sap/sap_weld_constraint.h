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

 Consider a frame P on an object A and a frame Q on an object B. This constraint
 is modeled with 6 constraint equations, which can be decomposed into a
 rotational component (3 equations) and a translational component (3 equations).

 Let a_PQ = θ⋅k be the Euler vector representation of the relative rotation
 R_PQ, Q's orientation in P. Let p = p_PoQo, Qo's position relative to Po. This
 constraint penalizes the full constraint function, g, to impose:
   g = (a_PQ, p_PoQo) = 0 ∈ ℝ⁶

 We note that this constraint enjoys the following properties:
   1. the formulation is frame invariant, and thus,
   2. the resulting impulses conserve angular momentum. Moreover,
   3. the resulting impulses satisfy Newton's third law.
 refer to implementation notes in MakeSapHolonomicConstraintKinematics()
 for details.

 The constraint impulse γ, which is composed of a rotational and translational
 component:
   γ = (γᵣ, γₜ) ∈ ℝ⁶
 This constraint impulse corresponds to a spatial impulse on B, applied at Bm,
 a point on B instantaneously coincident with midpoint M between P and Q:
   Γ_Bm_W = (γᵣ, γₜ)
 and likewise a spatial impulse on A, applied at Am, a point on A
 instantaneously coincident with midpoint M between P and Q:
   Γ_Am_W = -(γᵣ, γₜ)

 N.B. See DoAccumulateSpatialImpulses() for a discussion of where this
 interpretation of γ comes from.

 DoAccumulateSpatialImpulses() shifts these impulses to the body frame origins,
 reporting Γ_Bo_W and Γ_Ao_W, respectively.

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
       @param[in] J_W_AmBm Jacobian for the relative spatial velocity V_W_AmBm
         of Am and Bm, material points on A and B respectively, both coincident
         with M, the midpoint of P and Q.

       @note Thus far only dense Jacobian blocks are supported, i.e. rigid body
       applications in mind.
       @throws std::exception If J_W_AmBm.blocks_are_dense() == false.
    */
    Kinematics(int objectA, RigidTransform<T> X_WP, Vector3<T> p_AP_W,
               int objectB, RigidTransform<T> X_WQ, Vector3<T> p_BQ_W,
               SapConstraintJacobian<T> J_AmBm_W);

    int objectA() const { return objectA_; }
    const RigidTransform<T>& X_WP() const { return X_WP_; }
    const Vector3<T>& p_AP_W() const { return p_AP_W_; }
    int objectB() const { return objectB_; }
    const RigidTransform<T>& X_WQ() const { return X_WQ_; }
    const Vector3<T>& p_BQ_W() const { return p_BQ_W_; }
    const SapConstraintJacobian<T>& jacobian() const { return J_; }

    const Vector3<T>& p_PoQo_W() const { return p_PoQo_W_; }
    const Vector3<T>& a_PQ_W() const { return a_PQ_W_; }

    bool operator==(const Kinematics& other) const;

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

    /* Jacobian that defines the spatial velocity V_W_AmBm of Bm relative to Am,
     * expressed in the world frame. That is, V_W_AmBm = J⋅v. */
    SapConstraintJacobian<T> J_;

    /* Position vector from Po to point Qo expressed in the world frame. */
    Vector3<T> p_PoQo_W_;

    /* Rotation relating frame P to frame Q expressed as the axis angle
     * product a = θ⋅k expressed in the world frame. */
    Vector3<T> a_PQ_W_;
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
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;

  Kinematics kinematics_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
