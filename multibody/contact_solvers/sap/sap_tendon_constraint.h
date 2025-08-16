#pragma once

#include <memory>
#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Structure to store data needed for SapTendonConstraint computations.
 @tparam_nonsymbolic_scalar */
template <typename T>
struct SapTendonConstraintData {
  // Values stored in this struct remain const after construction.
  struct InvariantData {
    VectorX<T> H;      // Diagonal Hessian factor.
    VectorX<T> v_hat;  // Velocity bias, v̂.
  };
  InvariantData invariant_data;

  VectorX<T> v;        // Constraint velocity.
  T cost{};            // Cost ℓ(v)
  VectorX<T> impulse;  // Impulse γ(v) = −∂ℓ(v)/∂v.
  VectorX<T> hessian;  // (Diagonal) of Hessian G = −∂γ(v)/∂v = ∂²ℓ(v)/∂v².
};

/* Implements a tendon constraint for the SAP solver.

 This constraint implements the concept of an abstract "fixed" tendon as
 described in the MuJoCo model documentation:

 https://mujoco.readthedocs.io/en/stable/XMLreference.html#tendon-fixed

 Though inspired to provide the same capability its MuJoCo counterpart does,
 here we implement it in terms of the theory developed in [Castro et al. 2024],
 with a true physical parameterization of compliance parameters. See discussion
 in Drake's issue #22664 for further information on artifacts introduced by
 MuJoCo, which we solve here.

 The length of a tendon is defined as an affine function of the configuration:

   l(q) = aᵀ⋅q + offset ∈ ℝ

 Where it is assumed that the components of `a` have units such that l(q) has
 units of either meters (m) or radians (rad).

 This class imposes a set of two unilateral constraints such that this length
 stays within lower and upper limits, lₗ and lᵤ respectively:

   lₗ ≤ l(q) ≤ lᵤ

 Constraint kinematics:
  This constraint assumes that for any non-zero component aᵢ of a, the rate of
  change of the corresponding configuration qᵢ exactly equals its corresponding
  generalized velocities, i.e. q̇ᵢ = vᵢ

  The tendon constraint defines a constraint function g(q) ∈ ℝ² as:
    g = |l  - lₗ|   for the lower limit and,
        |lᵤ - l |   for the upper limit
  such that g(q) < 0 (component-wise) if the constraint is violated. The
  constraint velocity therefore is:
    ġ = | dl(q)/dt| = | aᵀ⋅q̇| = | aᵀ⋅v|
        |-dl(q)/dt|   |-aᵀ⋅q̇|   |-aᵀ⋅v|
  And therefore the constraint Jacobian is:
    J = | aᵀ|
        |-aᵀ|

  If one of the limits is infinite (lₗ = -∞ or lᵤ = +∞) then only one of the
  equations is considered (the one with finite bound) and g(q) ∈ ℝ i.e.
  num_constraint_equations() = 1.

 Compliant impulse:
  The (possibly) two constraint equations are independent so we can consider
  scalar equations for the components of the impulse γ = (γₗ, γᵤ) ∈ ℝ². The same
  analysis applies for both γₗ and γᵤ so we will drop the subscript in all
  subsequent equations. This constraint in the SAP formulation models a
  compliant impulse γ according to:

    γ = δt⋅(−k⋅g − d⋅ġ)₊

  where δt is the time step used in the formulation, k is the constraint
  stiffness (in either N/m or N⋅m/rad), d is the dissipation (in either N⋅s/m or
  N⋅m⋅rad/s), and (x)₊ = max(0, x) component-wise. Dissipation is parameterized
  as d = tau_d⋅k, where tau_d is the "dissipation time scale". These impulses
  are strictly positive (e.g. force is only applied to bring the length within
  limits).

  @warning Note though that for non-zero dissipation this constraint may apply
  non-physical "action at a distance" when g > 0 and ġ < 0. The distance at
  which these forces appear is directly proportional to both tau_d and ġ, so
  care must be taken to set dissipation appropriately such that the effect is
  not pronounced (or within an acceptable tolerance for the application at
  hand). For very stiff constraints (those within the "near-rigid" regime)
  tau_d ∝ δt, thus the distance at which these non-physical forces appear is
  also proportional to δt. This is typically acceptable for target applications.
  However, to avoid this artifact completely, we recommend only adding the
  constraint when g₀ < 0. This is the strategy we employ internally.

  We approximate the constraint function g as:
    g ≈ g₀ + δt⋅v
  where v = ġ is the constraint velocity and  g₀ the constraint function
  evaluated at q₀. Plugging into the expression for γ gives us a conditional
  expression for γ as a function of v:

           ⎧  H⋅(v̂ - v)   if v < v̂
    γ(v) = ⎨
           ⎩     0        if v ≥ v̂

  where:

    H = k⋅δt⋅(tau_d + δt)
    v̂ = -g₀ / (tau_d + δt)

  Which lets us conveniently define the constraint cost as:

           ⎧  (H/2)⋅(v̂ - v)²    if v < v̂
    ℓ(v) = ⎨
           ⎩       0            if v ≥ v̂

  And the constraint Hessian as:

           ⎧  H   if v < v̂
    G(v) = ⎨
           ⎩  0   if v ≥ v̂

 [Castro et al., 2024] Castro A., Han X., Masterjohn J., 2024. Irrotational
   Contact Fields. Available at https://arxiv.org/abs/2312.03908.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapTendonConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing. */
  //@{
  SapTendonConstraint& operator=(const SapTendonConstraint&) = delete;
  SapTendonConstraint(SapTendonConstraint&&) = delete;
  SapTendonConstraint& operator=(SapTendonConstraint&&) = delete;
  //@}

  /* Numerical parameters that define the constraint. Refer to
   SapTendonConstraint's class documentation for details. */
  class Parameters {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

    /* Constructs a valid set of parameters.
     @param lower_limit Lower limit of the tendon length in [m] or [rad].
     @param upper_limit Upper limit of the tendon length in [m] or [rad].
     @param stiffness Constraint stiffness k in [N/m] or [N⋅m/rad].
     @param damping Constraint damping d in [N⋅s/m] or [N⋅m⋅rad/s].
     @param beta Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the
     constraint frequency ωᵢ for the i-th constraint is below the limit ωᵢ ≤
     2π/δt. That is, the period is limited to Tᵢ = β⋅δt. w corresponds to a
     diagonal approximation of the Delassuss operator for the constraint. See
     [Castro et al., 2022] for details.

     @pre lower_limit < +∞
     @pre upper_limit > -∞
     @pre at least one of lower_limit and upper_limit is finite.
     @pre lower_limit <= upper_limit
     @pre stiffness > 0
     @pre damping >= 0
     @pre beta > 0 */
    Parameters(const T& lower_limit, const T& upper_limit, const T& stiffness,
               const T& damping, double beta = 0.1);

    const T& lower_limit() const { return lower_limit_; }
    const T& upper_limit() const { return upper_limit_; }
    const T& stiffness() const { return stiffness_; }
    const T& damping() const { return damping_; }
    double beta() const { return beta_; }
    int num_finite_limits() const { return num_finite_limits_; }

    bool has_finite_lower_limit() const;
    bool has_finite_upper_limit() const;

    bool operator==(const Parameters&) const = default;

   private:
    T lower_limit_{};
    T upper_limit_{};
    T stiffness_{};
    T damping_{};
    double beta_{0.1};
    // Stores the number of finite limits (1 or 2).
    int num_finite_limits_{};
  };

  /* Class to store the kinematics of the the constraint in its current
   configuration, when it gets constructed. */
  class Kinematics {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Kinematics);

    /* Constructs a valid set of kinematics representing the tendon length
        l(q) = aᵀ⋅q + b
     where q0 and q1 are the components of q(0) corresponding to clique0
     and clique1 and a0 and a1 the components of a corresponding to clique0
     and clique1 respectively.

     @pre clique0 >= 0
     @pre clique1 >= 0
     @pre clique0 != clique1
     @pre q0.size() == a0.size()
     @pre q1.size() == a1.size() */
    Kinematics(int clique0, int clique1, VectorX<T> q0, VectorX<T> q1,
               VectorX<T> a0, VectorX<T> a1, T offset);

    /* Single clique constructor.
     @pre clique0 >= 0
     @pre q0.size() == a0.size() */
    Kinematics(int clique0, VectorX<T> q0, VectorX<T> a0, T offset);

    int num_cliques() const { return clique1_.has_value() ? 2 : 1; }

    int clique0() const { return clique0_; }
    int clique1() const {
      DRAKE_THROW_UNLESS(num_cliques() > 1);
      return *clique1_;
    }
    int clique0_nv() const { return q0_.size(); }
    int clique1_nv() const {
      DRAKE_THROW_UNLESS(num_cliques() > 1);
      return q1_.size();
    }

    const VectorX<T>& q0() const { return q0_; }
    const VectorX<T>& q1() const {
      DRAKE_THROW_UNLESS(num_cliques() > 1);
      return q1_;
    }
    const VectorX<T>& a0() const { return a0_; }
    const VectorX<T>& a1() const {
      DRAKE_THROW_UNLESS(num_cliques() > 1);
      return a1_;
    }

    const T& offset() const { return offset_; }

    bool operator==(const Kinematics& other) const = default;

   private:
    /* Index of clique0. */
    int clique0_{};
    /* Index of clique1. std::nullopt if only one clique is involved. */
    std::optional<int> clique1_;
    /* q(0) of clique 0. */
    VectorX<T> q0_;
    /* q(0) of clique 1. */
    VectorX<T> q1_;
    /* Vector of coefficients for clique 0. */
    VectorX<T> a0_;
    /* Vector of coefficients for clique 1. */
    VectorX<T> a1_;
    /* Constant tendon length offset. */
    T offset_{};
  };

  /* Constructs a tendon constraint given its kinematics and parameters.
   If one of the limits is infinite (qₗ = -∞ or qᵤ = +∞) then only one of the
   equations is considered (the one with finite bound) and g(q) ∈ ℝ i.e.
   num_constraint_equations() = 1.
   @param[in] parameters Parameters of the constraint.
   @param[in] kinematics Kinematics of the constraint. */
  SapTendonConstraint(Parameters parameters, Kinematics kinematics);

  /* Returns the Parameters object used to construct this constraint. */
  const Parameters& parameters() const { return parameters_; }

  /* Returns the Kinematics object used to construct this constraint. */
  const Kinematics& kinematics() const { return kinematics_; }

  /* Returns the value of the constraint function computed at construction. At
   construction, Parameters can specify limits that are infinite (-∞ for lower
   and ∞ for upper), indicating there is no limit. Therefore, this constraint
   will implement a constraint function that can have size two (both limits
   finite) or size one (one of the limits is infinite).
   Thus if the lower limit is finite, its value is stored at
   constraint_function().head(1) and likewise if the upper limit is finite, its
   value is stored at constraint_function().tail(1). */
  const VectorX<T>& constraint_function() const { return g_; }

  /* Computes the constraint function g(q0) as a function of the parameters and
   the configuration given in `kinematics`. */
  static VectorX<T> CalcConstraintFunction(const Parameters& parameters,
                                           const Kinematics& kinematics);

 private:
  /* Private copy construction is enabled to use in the implementation of
   DoClone(). */
  SapTendonConstraint(const SapTendonConstraint&) = default;

  /* Computes the constraint Jacobian, independent of the configuration for this
   constraint. */
  static SapConstraintJacobian<T> CalcConstraintJacobian(
      const Parameters& parameters, const Kinematics& kinematics);

  /* Implementations of SapConstraint NVI functions. */
  std::unique_ptr<AbstractValue> DoMakeData(
      const T& dt,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const final;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& v,
                  AbstractValue* abstract_data) const final;
  T DoCalcCost(const AbstractValue& abstract_data) const final;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const final;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const final;
  std::unique_ptr<SapConstraint<T>> DoClone() const final;
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;
  void DoAccumulateGeneralizedImpulses(
      int c, const Eigen::Ref<const VectorX<T>>& gamma,
      EigenPtr<VectorX<T>> tau) const final;
  // no-op for this constraint.
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final;

  VectorX<T> g_;  // Constraint function g. See CalcConstraintFunction().
  Parameters parameters_;
  Kinematics kinematics_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
