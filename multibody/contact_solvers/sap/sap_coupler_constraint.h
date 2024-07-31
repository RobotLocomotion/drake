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

/* Implements a SAP (compliant) coupler constraint between two dofs.
 Given positions of dof 0 and 1 as q₀ and q₁, respectively, this constraint
 enforces:
   q₀ = ρ⋅q₁ + Δq
 where ρ is the gear ratio and Δq is a fixed offset.

 More precisely, this constraint enforces a single holonomic constraint
 equation:
   g = q₀ - ρ⋅q₁ - Δq = 0
 which produces a constraint impulse γ ∈ ℝ.

 The resulting generalized impulse on dof 0 is:
   j₀ = γ
 And the generalized impulse on dof 1 is:
   j₁ = −ργ

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapCouplerConstraint final : public SapHolonomicConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing. */
  //@{
  SapCouplerConstraint& operator=(const SapCouplerConstraint&) = delete;
  SapCouplerConstraint(SapCouplerConstraint&&) = delete;
  SapCouplerConstraint& operator=(SapCouplerConstraint&&) = delete;
  //@}

  /* Struct to store the kinematics of the the constraint in its current
   configuration, when it gets constructed. */
  struct Kinematics {
    bool operator==(const Kinematics&) const = default;

    /* Index of clique 0. */
    int clique0;
    /* Clique local index of dof 0. */
    int clique_dof0;
    /* Clique 0 number of velocities. */
    int clique_nv0;
    /* Position of dof 0. */
    T q0;
    /* Index of clique 1. */
    int clique1;
    /* Clique local index of dof 1. */
    int clique_dof1;
    /* Clique 1 number of velocities. */
    int clique_nv1;
    /* Position of dof 1. */
    T q1;
    /* Gear ratio ρ. */
    T gear_ratio;
    /* Constraint function bias Δq. */
    T offset;
  };

  /* Constructs a coupler constraint given its kinematics in a particular
   configuration. */
  explicit SapCouplerConstraint(Kinematics kinematics);

  const Kinematics& kinematics() const { return kinematics_; }

 private:
  /* Private copy construction is enabled to use in the implementation of
     DoClone(). */
  SapCouplerConstraint(const SapCouplerConstraint&) = default;

  /* Accumulates generalized impulses applied by this constraint on the c-th
   clique.
   @param[in] c The c-th clique of index clique(c).
   @param[in] gamma Impulses for this constraint, of size
   num_constraint_equations().
   @param[out] tau On output this function will accumulate the generalized
   impulses applied by this constraint on the c-th clique.
  */
  void DoAccumulateGeneralizedImpulses(
      int c, const Eigen::Ref<const VectorX<T>>& gamma,
      EigenPtr<VectorX<T>> tau) const final;

  /* No-op for this constraint. */
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final {}

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
    return std::unique_ptr<SapCouplerConstraint<T>>(
        new SapCouplerConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;

  Kinematics kinematics_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
