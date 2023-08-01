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

/* Implements a SAP (compliant) coupler constraint between two single-dof
 joints.

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

  /* ComplianceParameters that define the constraint's compliance. */
  class ComplianceParameters {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ComplianceParameters);

    /* Stiffness k and dissipation time scale tau_d. */
    ComplianceParameters(T stiffness, T dissipation_time_scale);

    const T& stiffness() const { return stiffness_; }

    const T& dissipation_time_scale() const { return dissipation_time_scale_; }

   private:
    T stiffness_;
    T dissipation_time_scale_;
  };

  /* Struct to store the kinematics of the the constraint in its current
   configuration, when it gets constructed. */
  struct Kinematics {
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
    /* Ratio relating dof 0 to dof 1. */
    T gear_ratio;
    /* Constant bias added to constraint function. */
    T offset;
  };

  /* Constructs a coupler constraint given its kinematics in a particular
   configuration and the set of parameters that define the constraint. */
  SapCouplerConstraint(Kinematics kinematics, ComplianceParameters parameters);

  const ComplianceParameters& compliance_parameters() const {
    return parameters_;
  }

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

  /* Helper used at construction. Given parameters `p` for a
   SapCouplerConstraint, this method makes the parameters needed by the base
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
    return std::unique_ptr<SapCouplerConstraint<T>>(
        new SapCouplerConstraint<T>(*this));
  }

  Kinematics kinematics_;
  ComplianceParameters parameters_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
