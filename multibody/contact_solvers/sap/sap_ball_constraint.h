#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Implements a SAP (compliant) ball constraint between two points.

 To be more precise, consider a point P on an object A and point Q on an object
 B. Working in the world frame W, this constraint penalizes non-coincident P and
 Q with the constraint function:
   g = p_WQ - p_WP = 0
 with corresponding constraint velocity:
   ġ = vc(v) = v_W_PQ

 N.B. Only when P and Q are coincident is their relative velocity independent
 of which frame it is measured in. This means that the way we model this
 constraint lacks frame-invariance. It is unclear at this time if choosing a
 particular frame leads to a better approximation, conditioning, etc. We choose
 the world frame for convenience.

 This leads to 3 holonomic constraint equations, producing a constraint
 impulse, γ ∈ ℝ³. The impulse on B applied at Q is:
   γ_Bq_W = γ
 and the reaction force on A applied at Q is:
   γ_Ap_W = -γ

 N.B. See DoAccumulateSpatialImpulses() for a discussion of where this
 interpretation of γ comes from. In general when P and Q are not coincident this
 formulation does NOT conserve angular momentum, introducing a small moment of
 order O(‖γ‖⋅‖p_PQ‖).

 DoAccumulateSpatialImpulses() shifts these impulses to the body frame origins,
 reporting spatial impulses Γ_Bo_W and Γ_Ao_W, respectively.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapBallConstraint final : public SapHolonomicConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing. */
  //@{
  SapBallConstraint& operator=(const SapBallConstraint&) = delete;
  SapBallConstraint(SapBallConstraint&&) = delete;
  SapBallConstraint& operator=(SapBallConstraint&&) = delete;
  //@}

  /* Class to store the kinematics of the the constraint in its current
   configuration, when it gets constructed. */
  class Kinematics {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Kinematics);

    /* @param[in] objectA
         Index of the physical object A on which point P attaches.
       @param[in] p_WP Position of point P in the world frame.
       @param[in] p_AP_W Position of point P in A, expressed in the world frame.
       @param[in] objectB
         Index of the physical object B on which point Q attaches.
       @param[in] p_WQ Position of point Q in the world frame.
       @param[in] p_BQ_W Position of point Q in B, expressed in the world frame.
       @param[in] J_ApBq_W Jacobian for the relative velocity v_ApBq_W. */
    Kinematics(int objectA, Vector3<T> p_WP, Vector3<T> p_AP_W, int objectB,
               Vector3<T> p_WQ, Vector3<T> p_BQ_W,
               SapConstraintJacobian<T> J_ApBq_W)
        : objectA_(objectA),
          p_WP_(std::move(p_WP)),
          p_AP_W_(std::move(p_AP_W)),
          objectB_(objectB),
          p_WQ_(std::move(p_WQ)),
          p_BQ_W_(std::move(p_BQ_W)),
          J_(std::move(J_ApBq_W)) {
      // Thus far only dense Jacobian blocks are supported, i.e. rigid body
      // applications in mind.
      DRAKE_THROW_UNLESS(J_.blocks_are_dense());
    }

    int objectA() const { return objectA_; }
    const Vector3<T>& p_WP() const { return p_WP_; }
    const Vector3<T>& p_AP_W() const { return p_AP_W_; }
    int objectB() const { return objectB_; }
    const Vector3<T>& p_WQ() const { return p_WQ_; }
    const Vector3<T>& p_BQ_W() const { return p_BQ_W_; }
    const SapConstraintJacobian<T>& jacobian() const { return J_; }

    bool operator==(const Kinematics&) const = default;

   private:
    /* Index to a physical object A. */
    int objectA_;

    /* Position of point P in world. */
    Vector3<T> p_WP_;

    /* Position of point P in A, expressed in the world. */
    Vector3<T> p_AP_W_;

    /* Index to a physical object B. */
    int objectB_;

    /* Position of point Q in world. */
    Vector3<T> p_WQ_;

    /* Position of point Q in B, expressed in the world. */
    Vector3<T> p_BQ_W_;

    /* Jacobian that defines the velocity v_ApBq_W of point P relative to point
     Q, expressed in the world frame. That is, v_ApBq_W = J⋅v. */
    SapConstraintJacobian<T> J_;
  };

  /* Constructs a ball constraint given its kinematics in a particular
   configuration. */
  explicit SapBallConstraint(Kinematics kinematics);

  const Kinematics& kinematics() const { return kinematics_; }

 private:
  /* Private copy construction is enabled to use in the implementation of
     DoClone(). */
  SapBallConstraint(const SapBallConstraint&) = default;

  // no-op for this constraint.
  void DoAccumulateGeneralizedImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                       EigenPtr<VectorX<T>>) const final {}

  /* Accumulates generalized forces applied by this constraint on the i-th
   object.
   @param[in] i Object index. As defined at construction i = 0 corresponds to
   object A and i = 1 corresponds to object B.
   @param[in] gamma A vector of size 3, with the constraint impulse.
   @param[out] F On output, this method accumulates the total spatial impulse
   applied by this constraint on the i-th object.
   @pre 0 ≤ i < 2.
   @pre gamma is a vector of size 3.
   @pre F is not nullptr. */
  void DoAccumulateSpatialImpulses(int i,
                                   const Eigen::Ref<const VectorX<T>>& gamma,
                                   SpatialForce<T>* F) const final;

  /* Helper used at construction. This method makes the parameters needed by the
   base class SapHolonomicConstraint. */
  static typename SapHolonomicConstraint<T>::Parameters
  MakeSapHolonomicConstraintParameters();

  /* Helper used at construction. Makes the constraint function and Jacobian
   needed to initialize the base class SapHolonomicConstraint.
   @returns Holonomic constraint kinematics needed at construction of the
   parent SapHolonomicConstraint. */
  static typename SapHolonomicConstraint<T>::Kinematics
  MakeSapHolonomicConstraintKinematics(const Kinematics& kinematics);

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapBallConstraint<T>>(
        new SapBallConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;

  Kinematics kinematics_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
