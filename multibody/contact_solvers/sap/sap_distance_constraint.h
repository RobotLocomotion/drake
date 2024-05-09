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

template <typename T>
class SapDistanceConstraint;

/* Implements a SAP (compliant) distance constraint between two points. With
 finite compliance, this constraint can be used to model a linear spring between
 two points.

 To be more precise, consider a point P on an object A and point Q on an object
 B. With d the distance between points P and Q and ḋ its rate of change, this
 SAP constraint models the constraint impulse as γ ∈ ℝ:
   γ = −k⋅(d−ℓ) − c⋅ḋ
 where ℓ is the "free length" of the constraint, and k and c are the stiffness
 and damping coefficients respectively.

 With p̂ the unit vector from P to Q, the force vector on B applied at Q is:
   f_Bq = γ⋅p̂
 and the reaction force on A applied at P is:
   f_Ap = -γ⋅p̂

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapDistanceConstraint final : public SapHolonomicConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing. */
  //@{
  SapDistanceConstraint& operator=(const SapDistanceConstraint&) = delete;
  SapDistanceConstraint(SapDistanceConstraint&&) = delete;
  SapDistanceConstraint& operator=(SapDistanceConstraint&&) = delete;
  //@}

  /* ComplianceParameters that define the constraint's compliance. */
  class ComplianceParameters {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ComplianceParameters);

    /* Stiffness k and damping c. */
    ComplianceParameters(T stiffness, T damping);

    bool operator==(const ComplianceParameters&) const = default;

    const T& stiffness() const { return stiffness_; }

    const T& damping() const { return damping_; }

   private:
    T stiffness_;
    T damping_;
  };

  /* Class to store the kinematics of the the constraint in its current
   configuration, when it gets constructed. */
  class Kinematics {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Kinematics);

    /* @param[in] objectA
         Index of the physical object A on which point P attaches.
       @param[in] p_WP_W Position of point P in the world frame.
       @param[in] p_AP_W Position of point P in A, expressed in the world frame.
       @param[in] objectB
         Index of the physical object B on which point Q attaches.
       @param[in] p_WQ_W Position of point Q in the world frame.
       @param[in] p_BQ_W Position of point Q in B, expressed in the world frame.
       @param[in] length The constraint's length.
       @param[in] J_ApBq_W Jacobian for the relative velocity v_ApBq_W. */
    Kinematics(int objectA, Vector3<T> p_WP_W, Vector3<T> p_AP_W, int objectB,
               Vector3<T> p_WQ_W, Vector3<T> p_BQ_W, T length,
               SapConstraintJacobian<T> J_ApBq_W)
        : objectA_(objectA),
          p_WP_(std::move(p_WP_W)),
          p_AP_W_(std::move(p_AP_W)),
          objectB_(objectB),
          p_WQ_(std::move(p_WQ_W)),
          p_BQ_W_(std::move(p_BQ_W)),
          length_(std::move(length)),
          J_(std::move(J_ApBq_W)) {
      // Thus far only dense Jacobian blocks are supported, i.e. rigid body
      // applications in mind.
      DRAKE_THROW_UNLESS(J_.blocks_are_dense());

      const Vector3<T> p_PQ_W = p_WQ_ - p_WP_;
      distance_ = p_PQ_W.norm();
      // Verify the distance did not become ridiculously small. We use the
      // constraint's fixed length as a reference.
      constexpr double kMinimumDistance = 1.0e-7;
      constexpr double kRelativeDistance = 1.0e-2;
      if (distance_ < kMinimumDistance + kRelativeDistance * length_) {
        throw std::logic_error(
            fmt::format("The distance is {}. This is nonphysically small when "
                        "compared to the free length of the constraint, {}. ",
                        distance_, length_));
      }
      p_hat_W_ = p_PQ_W / distance_;
    }

    int objectA() const { return objectA_; }
    const Vector3<T>& p_WP() const { return p_WP_; }
    const Vector3<T>& p_AP_W() const { return p_AP_W_; }
    int objectB() const { return objectB_; }
    const Vector3<T>& p_WQ() const { return p_WQ_; }
    const Vector3<T>& p_BQ_W() const { return p_BQ_W_; }
    const T& length() const { return length_; }
    const T& distance() const { return distance_; }
    const SapConstraintJacobian<T>& jacobian() const { return J_; }
    const Vector3<T>& p_hat_W() const { return p_hat_W_; }

    bool operator==(const Kinematics&) const = default;

   private:
    /* Index to a physical object A. */
    int objectA_;

    /* Position of point P in the world. */
    Vector3<T> p_WP_;

    /* Position of point P relative to A. */
    Vector3<T> p_AP_W_;

    /* Index to a physical object B. */
    int objectB_;

    /* Position of point Q in the world. */
    Vector3<T> p_WQ_;

    /* Position of point Q relative to B. */
    Vector3<T> p_BQ_W_;

    /* Fixed length of the constraint. */
    T length_;

    /* Jacobian that defines the velocity v_ApBq_W of point P relative to point
     Q, expressed in the world frame. That is, v_ApBq_W = J⋅v. */
    SapConstraintJacobian<T> J_;

    /* Distance between point P and Q. */
    T distance_;

    /* Versor pointing from point P to point Q, expressed in the world frame W.
     */
    Vector3<T> p_hat_W_;
  };

  /* Constructs a distance constraint given its kinematics in a particular
   configuration and the set of parameters that define the constraint. */
  SapDistanceConstraint(Kinematics kinematics, ComplianceParameters parameters);

  /* The constraint's length. */
  const T& length() const { return kinematics_.length(); }

  const ComplianceParameters& compliance_parameters() const {
    return parameters_;
  }

  const Kinematics& kinematics() const { return kinematics_; }

 private:
  /* Private copy construction is enabled to use in the implementation of
     DoClone(). */
  SapDistanceConstraint(const SapDistanceConstraint&) = default;

  // no-op for this constraint.
  void DoAccumulateGeneralizedImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                       EigenPtr<VectorX<T>>) const final {}

  /* Accumulates generalized forces applied by this constraint on the i-th
   object.
   @param[in] i Object index. As defined at construction i = 0 corresponds to
   object A and i = 1 corresponds to object B.
   @param[in] gamma A vector of size 1, with the (scalar) constraint impulse.
   @param[out] F On output, this method accumulates the total spatial impulse
   applied by this constraint on the i-th object.
   @pre 0 ≤ i < 2.
   @pre gamma is a vector of size one.
   @pre F is not nullptr. */
  void DoAccumulateSpatialImpulses(int i,
                                   const Eigen::Ref<const VectorX<T>>& gamma,
                                   SpatialForce<T>* F) const final;

  /* Helper used at construction. Given parameters `p` for a
   SapDistanceConstraint, this method makes the parameters needed by the base
   class SapHolonomicConstraint. */
  static typename SapHolonomicConstraint<T>::Parameters
  MakeSapHolonomicConstraintParameters(const ComplianceParameters& p);

  /* Helper used at construction. Makes the constraint function and Jacobian
   needed to initialize the base class SapHolonomicConstraint.
   @returns Holonomic constraint kinematics needed at construction of the
   parent SapHolonomicConstraint. */
  static typename SapHolonomicConstraint<T>::Kinematics
  MakeSapHolonomicConstraintKinematics(const Kinematics& kinematics);

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapDistanceConstraint<T>>(
        new SapDistanceConstraint<T>(*this));
  }

  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;

  Kinematics kinematics_;
  ComplianceParameters parameters_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
