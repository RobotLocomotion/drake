#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Structure to store data needed for SapLimitConstraint computations. */
template <typename T>
struct SapLimitConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapLimitConstraintData);

  /* Constructs data for a SapLimitConstraint.
     @param R Regularization parameters.
     @param vn_hat Bias term. */
  SapLimitConstraintData(VectorX<T> R, VectorX<T> v_hat) {
    const int nk = R.size();
    parameters_.R.resize(nk);
    parameters_.R_inv.resize(nk);
    parameters_.v_hat.resize(nk);
    parameters_.R = R;
    parameters_.R_inv = R.cwiseInverse();
    parameters_.v_hat = v_hat;
    vc_.resize(nk);
    y_.resize(nk);
    gamma_.resize(nk);
    dPdy_.resize(nk, nk);
  }

  /* Regularization R. */
  const VectorX<T>& R() const { return parameters_.R; }

  /* Inverse of the regularization, R⁻¹. */
  const VectorX<T>& R_inv() const { return parameters_.R_inv; }

  /* Constraint bias. */
  const VectorX<T>& v_hat() const { return parameters_.v_hat; }

  /* Const access. */
  const VectorX<T>& vc() const { return vc_; }
  const VectorX<T>& y() const { return y_; }
  const VectorX<T>& gamma() const { return gamma_; }
  const MatrixX<T>& dPdy() const { return dPdy_; }

  /* Mutable access. */
  VectorX<T>& mutable_vc() { return vc_; }
  VectorX<T>& mutable_y() { return y_; }
  VectorX<T>& mutable_gamma() { return gamma_; }
  MatrixX<T>& mutable_dPdy() { return dPdy_; }

 private:
  struct ConstParameters {
    VectorX<T> R;  // Regularization R.
    VectorX<T> R_inv;
    VectorX<T> v_hat;  // Constraint velocity bias.
  };
  ConstParameters parameters_;

  VectorX<T> vc_;
  VectorX<T> y_;      // Un-projected impulse y = −R⁻¹⋅(vc−v̂)
  VectorX<T> gamma_;  // Impulse.
  MatrixX<T> dPdy_;   // Gradient of the projection γ = P(y) w.r.t. y.
};

/* Implements limit constraints for the SAP solver, [Castro et al., 2021]. This
 constraint is used to impose a (compliant, see below) limit on the i-th degree
 of freedom (DOF) of a given clique in a SapContactModel. This constraint
 assumes that the rate of change of the i-th configuration exactly equals its
 corresponding generalized velocities, i.e. q̇ᵢ = vᵢ. This is specially true for
 1-DOF joints such as revolute and prismatic.

 Constraint kinematics:
  We consider the i-th DOF of a clique with m DOFs.
  We denote the configuration with qᵢ its lower limit with qₗ and its upper
  limit with qᵤ, where qₗ < qᵤ. The limit constraint defines a constraint
  function g(q) ∈ ℝ² as:
    g = |qᵢ - qₗ|   for the lower limit and,
        |qᵤ - qᵢ|   for the upper limit
  such that g(qᵢ) < 0 (componentwise) if the constraint is violated. The
  constraint velocity therefore is:
    ġ = | q̇ᵢ|
        |-q̇ᵢ|
  And therefore the constraint Jacobian is:
    J = | eᵢᵀ|
        |-eᵢᵀ|
  where eᵢ is the i-th element of the standard basis of ℝᵐ whose components are
  all zero, except for the i-th component that equals to 1.

  If one of the limits is infinite (qₗ = -∞ or qᵤ = +∞) then only one of the
  equations is considered (the one with finite bound) and g(q) ∈ ℝ i.e.
  num_constraint_equations() = 1.

 Compliant impulse:
  Limit constraints in the SAP formulation model a compliant impulse γ according
  to:
    y/dt = −k⋅g−d⋅ġ
    γ/δt = P(y)
    P(y) = (y)₊
  where δt is the time step used in the formulation, k is the constraint
  stiffness (in N/m), d is the dissipation (in N⋅s/m) and (x)₊ = max(0, x),
  componentwise. Dissipation is parameterized as d = tau_d⋅k, where tau_d is the
  "dissipation time scale". Notice that these impulses are positive when the
  constraint is active and zero otherwise.
  For this constraint the components of γ = [γₗ, γᵤ]ᵀ are constrained to live in
  ℝ⁺ and therefore the projection can trivially be computed analytically as
  P(y) = (y)₊, independent of the compliant regularization, see [Todorov, 2014].

 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107
 [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
   dynamics with contacts and constraints: Theory and implementation in mujoco.
   In 2014 IEEE International Conference on Robotics and Automation (ICRA) (pp.
   6054-6061). IEEE.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapLimitConstraint final : public SapConstraint<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapLimitConstraint);

  /* Numerical parameters that define the constraint. Refer to this class's
   documentation for details. */
  class Parameters {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

    /* Constructs a valid set of parameters.
     @pre lower_limit < +∞
     @pre upper_limit > -∞
     @pre at least one of lower_limit and upper_limit is finite.
     @pre lower_limit <= upper_limit
     @pre stiffness > 0
     @pre dissipation_time_scale >= 0
     @pre beta > 0 */
    Parameters(const T& lower_limit, const T& upper_limit, const T& stiffness,
               const T& dissipation_time_scale, double beta = 0.1);

    const T& lower_limit() const { return lower_limit_; }
    const T& upper_limit() const { return upper_limit_; }
    const T& stiffness() const { return stiffness_; }
    const T& dissipation_time_scale() const { return dissipation_time_scale_; }
    double beta() const { return beta_; }

   private:
    T lower_limit_;
    T upper_limit_;
    /* Contact stiffness k, in N/m. It must be strictly positive. */
    T stiffness_;
    /* Dissipation time scale tau_d, in seconds. It must be non-negative. */
    T dissipation_time_scale_;
    /* Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the constraint
     frequency ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ =
     β⋅δt. w corresponds to a diagonal approximation of the Delassuss operator
     for each contact. See [Castro et al., 2021] for details. */
    double beta_{0.1};
  };

  /* Constructs a limit constraint for DOF with index `clique_dof` within clique
   with index `clique` in a given SapContactProblem. If one of the limits is
   infinite (qₗ = -∞ or qᵤ = +∞) then only one of the equations is considered
   (the one with finite bound) and g(q) ∈ ℝ i.e. num_constraint_equations() = 1.
   @param[in] clique The clique involved in the contact. Must be non-negative.
   @param[in] clique_dof DOF in `clique` to be constrained. It must be in [0,
   clique_nv).
   @param[in] clique_nv Number of generalized velocities for `clique`.
   @param[in] q0 Current configuration of the constraint.
   @param[in] parameters Parameters of the constraint. */
  SapLimitConstraint(int clique, int clique_dof, int clique_nv, const T& q0,
                     Parameters parameters);

  const Parameters& parameters() const { return parameters_; }

  /* Returns the degree of freedom, DOF, that this constraint limits. Provided
   at construction. */
  int clique_dof() const { return clique_dof_; }

  /* Returns the position provided at construction. */
  const T& position() const { return q0_; }

  std::unique_ptr<SapConstraint<T>> Clone() const final;

 private:
  /* Computes the constraint function g(q0) as a function of q0 for given lower
   limit ql and upper limit qu.
   @pre lower_limit < +∞
   @pre upper_limit > -∞ */
  static VectorX<T> CalcConstraintFunction(const T& q0, const T& ql,
                                           const T& qu);

  /* Computes the constraint Jacobian, independent of the configuration for this
   constraint. */
  static MatrixX<T> CalcConstraintJacobian(int clique_dof, int clique_nv,
                                           const T& ql, const T& qu);

  void Project(const Eigen::Ref<const VectorX<T>>& y,
               EigenPtr<VectorX<T>> gamma, MatrixX<T>* dPdy = nullptr) const;

  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const override;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* data) const override;
  T DoCalcCost(const AbstractValue& abstract_data) const override;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const override;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const override;

  Parameters parameters_;
  int clique_dof_{-1};  // Initialized to an invalid value.
  T q0_{};              // position at the configuration from construction.
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
