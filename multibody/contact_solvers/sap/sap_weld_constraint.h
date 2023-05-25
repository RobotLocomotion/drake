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

TODO(joemasterjohn): Write up weld constraint derivation in unicode.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapWeldConstraint : public SapHolonomicConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Protected copy construction is enabled for sub-classes to use in their
    implementation of DoClone(). */
  //@{
  SapWeldConstraint& operator=(const SapWeldConstraint&) = delete;
  SapWeldConstraint(SapWeldConstraint&&) = delete;
  SapWeldConstraint& operator=(SapWeldConstraint&&) = delete;
  //@}

  /* Class to store the kinematics of the the constraint in its current
   configuration, when it gets constructed. */
  class Kinematics {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Kinematics);

    /* @param[in] objectA
         Index of the physical object A on which frame P attaches.
       @param[in] X_AP Pose of frame P in object A's frame.
       @param[in] X_WA Pose of object A's frame in the world frame.
       @param[in] objectB
         Index of the physical object B on which frame Q attaches.
       @param[in] X_BQ Pose of frame Q in object B's frame.
       @param[in] X_WB Pose of object B's frame in the world frame.
       @param[in] Jv_PQ_W Jacobian for the relative velocity v_PQ_W. */
    Kinematics(int objectA, RigidTransform<T> X_AP, RigidTransform<T> X_WA,
               int objectB, RigidTransform<T> X_BQ, RigidTransform<T> X_WB,
               SapConstraintJacobian<T> Jv_PQ_W);

    int objectA() const { return objectA_; }
    const RigidTransform<T>& X_AP() const { return X_AP_; }
    const RigidTransform<T>& X_WA() const { return X_WA_; }
    int objectB() const { return objectB_; }
    const RigidTransform<T>& X_BQ() const { return X_BQ_; }
    const RigidTransform<T>& X_WB() const { return X_WB_; }
    const SapConstraintJacobian<T>& jacobian() const { return J_; }

    const RigidTransform<T>& X_WN() const { return X_WN_; }
    const RigidTransform<T>& X_NA() const { return X_NA_; }
    const RigidTransform<T>& X_NB() const { return X_NB_; }
    const Vector3<T>& p_PoQo_N() const { return p_PoQo_N_; }
    const Vector3<T>& a_N() const { return a_N_; }

   private:
    /* Index to a physical object A. */
    int objectA_;

    /* Pose of frame P in object A's frame. */
    RigidTransform<T> X_AP_;

    /* Pose of object A's frame in the world frame. */
    RigidTransform<T> X_WA_;

    /* Index to a physical object B. */
    int objectB_;

    /* Pose of frame Q in object B's frame. */
    RigidTransform<T> X_BQ_;

    /* Pose of object B's frame in the world frame.  */
    RigidTransform<T> X_WB_;

    /* Jacobian that defines the spatial velocity V_PQ_W of frame P relative
     to point Q, expressed in the world frame. That is, v_ApBq_W = J⋅v. */
    SapConstraintJacobian<T> J_;

    /* Pose of intermediate frame N in the world frame. */
    RigidTransform<T> X_WN_;

    /* Pose of frame A in intermediate frame N. */
    RigidTransform<T> X_NA_;

    /* Pse of frame B in intermediate frame N. */
    RigidTransform<T> X_NB_;

    /* Position vector from Po to point Qo expressed in frame N. */
    Vector3<T> p_PoQo_N_;

    /* Rotation relating frame P to frame Q expressed as the axis angle
     * product a = θ⋅λ expressed in frame N */
    Vector3<T> a_N_;
  };

  /* Constructs a weld constraint given its kinematics in a particular
   configuration */
  SapWeldConstraint(Kinematics kinematics, const T& relaxation_time);

  const Kinematics& kinematics() const { return kinematics_; }

 private:
  /* Private copy construction is enabled to use in the implementation of
     DoClone(). */
  SapWeldConstraint(const SapWeldConstraint&) = default;

  /* no-op for this constraint. */
  void DoAccumulateGeneralizedImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                       EigenPtr<VectorX<T>>) const final {}

  /* Accumulates generalized forces applied by this constraint on the i-th
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

  static typename SapHolonomicConstraint<T>::Parameters
  MakeSapHolonomicConstraintParameters(const T& relaxation_time);

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
