#pragma once

#include <limits>
#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Structure to store data needed for SapFixedTendonConstraint computations.
 @tparam_nonsymbolic_scalar */
template <typename T>
struct SapFixedTendonConstraintData {
  // Values stored in this struct remain const after construction.
  struct InvariantData {
    VectorX<T> H;      // Hessian factor.
    VectorX<T> v_hat;  // Velocity bias, v̂.
  };
  InvariantData invariant_data;

  VectorX<T> v_;        // Constraint velocity.
  T cost_;              // Cost ℓ(v)
  VectorX<T> impulse_;  // Impulse γ(v) = −∂ℓ(v)/∂v.
  VectorX<T> hessian_;  // (Diagonal) of Hessian G = −∂γ(v)/∂v = ∂²ℓ(v)/∂v².
};

/* Implements a "fixed" tendon constraint for the SAP solver, [Castro et al.,
 2022].

 This constraint implements the concept of an abstract "fixed" tendon as
 described in the MuJoCo model documentation:
 https://mujoco.readthedocs.io/en/stable/XMLreference.html#tendon-fixed

 The length of a fixed tendon is defined as an affine function of the
 configuration:

   l(q) = aᵀ⋅q + offset ∈ ℝ

 Where it is assumed that the components of `a` have units such that l(q) has
 units of either meters (m) or radians (rad).

 This class imposes a set of uni-lateral constraints such that this length stays
 within lower and upper limits, lₗ and lᵤ respectively:

   lₗ ≤ l(q) ≤ lᵤ

 Constraint kinematics:
  This constraint assumes that for any non-zero component aᵢ of a, the rate of
  change of the corresponding configuration qᵢ exactly equals its corresponding
  generalized velocities, i.e. q̇ᵢ = vᵢ

  The fixed tendon constraint defines a constraint function g(q) ∈ ℝ² as:
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
  limits). Note though that for non-zero dissipation this constraint may apply
  non-physical "action at a distance" when g > 0 and ġ < 0. The distance at
  which these forces appear is directly proportional to both tau_d ang ġ, so
  care must be taken to set dissipation appropriately such that the effect is
  not pronounced (or within an acceptable tolerance for the application at
  hand). For very stiff constraints (those within the "near-rigid" regime)
  tau_d ∝ δt, thus the distance at which these non-physical forces appear is
  also proportional to δt. This is typically acceptable for target applications.

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

 [Castro et al., 2022] Castro A., Permenter F. and Han X., 2022. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapFixedTendonConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Protected copy construction is enabled for sub-classes to use in their
    implementation of DoClone(). */
  //@{
  SapFixedTendonConstraint& operator=(const SapFixedTendonConstraint&) = delete;
  SapFixedTendonConstraint(SapFixedTendonConstraint&&) = delete;
  SapFixedTendonConstraint& operator=(SapFixedTendonConstraint&&) = delete;
  //@}

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
     @pre damping >= 0
     @pre beta > 0 */
    Parameters(const T& lower_limit, const T& upper_limit, const T& stiffness,
               const T& damping, double beta = 0.1);

    const T& lower_limit() const { return lower_limit_; }
    const T& upper_limit() const { return upper_limit_; }
    const T& stiffness() const { return stiffness_; }
    const T& damping() const { return damping_; }
    double beta() const { return beta_; }

    bool operator==(const Parameters&) const = default;

   private:
    /* Constraint lower limit, in m. It must be < +∞. */
    T lower_limit_;
    /* Constraint upper limit, in m. It must be > -∞. */
    T upper_limit_;
    /* Constraint stiffness k, in N/m. It must be strictly positive. */
    T stiffness_;
    /* Constraint damping in N⋅s/m. It must be non-negative. */
    T damping_;
    /* Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the constraint
     frequency ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ =
     β⋅δt. w corresponds to a diagonal approximation of the Delassuss operator
     for each constraint. See [Castro et al., 2022] for details. */
    double beta_{0.1};
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
     @pre clique1 >= -1
     @pre clique0 != clique1
     @pre clique0_nv >= 0
     @pre clique1_nv >= 0 if clique1 >= 0
     @pre q0.size() == clique0_nv
     @pre q1.size() == clique1_nv if clique1 >= 0
     @pre a0.size() == clique0_nv
     @pre a0.size() == clique1_nv if clique1 >= 0 */
    Kinematics(int clique0, int clique1, int clique0_nv, int clique1_nv,
               VectorX<T> q0, VectorX<T> q1, VectorX<T> a0, VectorX<T> a1,
               T offset);

    /* Single clique constructor.
     @pre clique0 >= 0
     @pre clique0_nv >= 0
     @pre q0.size() == clique0_nv
     @pre a0.size() == clique0_nv */
    Kinematics(int clique0, int clique0_nv, VectorX<T> q0, VectorX<T> a0,
               T offset);

    int clique0() const { return clique0_; }
    int clique1() const { return clique1_; }
    int clique0_nv() const { return clique0_nv_; }
    int clique1_nv() const { return clique1_nv_; }

    const VectorX<T>& q0() const { return q0_; }
    const VectorX<T>& q1() const { return q1_; }
    const VectorX<T>& a0() const { return a0_; }
    const VectorX<T>& a1() const { return a1_; }

    const T& offset() const { return offset_; }

    bool operator==(const Kinematics&) const = default;

   private:
    /* Index of clique0. */
    int clique0_;
    /* Index of clique1. -1 if only one clique is involved. */
    int clique1_{-1};
    /* nv of clique 0. */
    int clique0_nv_;
    /* nv of clique 1. -1 if clique1_ == -1. */
    int clique1_nv_{-1};
    /* q(0) of clique 0. */
    VectorX<T> q0_;
    /* q(0) of clique 1. */
    VectorX<T> q1_;
    /* Vector of coefficients for clique 0. */
    VectorX<T> a0_;
    /* Vector of coefficients for clique 1. */
    VectorX<T> a1_;
    /* Constant tendon length offset. */
    T offset_;
  };

  /* Constructs a fixed tendon constraint given its kinematics and parameters.
   If one of the limits is infinite (qₗ = -∞ or qᵤ = +∞) then only one of the
   equations is considered (the one with finite bound) and g(q) ∈ ℝ i.e.
   num_constraint_equations() = 1.
   @param[in] parameters Parameters of the constraint.
   @param[in] kinematics Kinematics of the constraint. */
  SapFixedTendonConstraint(Parameters parameters, Kinematics kinematics);

  /* Returns the Parameters object used to construct this constraint. */
  const Parameters& parameters() const { return parameters_; }

  /* Returns the Kinematics object used to construct this constraint. */
  const Kinematics& kinematics() const { return kinematics_; }

  /* Returns the value of the constraint function computed at construction. At
   construction, Parameters can specify limits that are infinite (-∞ for lower
   and ∞ for upper), indicating there is no limit. Therefore, this constraint
   will implement a constraint function that can have size two (both limits
   finite) or size one (one of the limits is infinite).
   Therefore the returned vector stores the value of the constraint function as:
     1. the first entry contains the value of the lower limit constraint
        function iff the lower limit is finite.
     2. The next entry contains the value of the upper limit constraint
        function iff the upper limit is finite.
   There is no information in the returned value on which limits were included.
   That information however is known to the client code that provided the
   initial constraint parameters. */
  const VectorX<T>& constraint_function() const { return g_; }

  /* Computes the constraint function g(q0) as a function of the parameters and
   the configuration given in `kinematics`. */
  static VectorX<T> CalcConstraintFunction(const Parameters& parameters,
                                           const Kinematics& kinematics);

 private:
  /* Private copy construction is enabled to use in the implementation of
   DoClone(). */
  SapFixedTendonConstraint(const SapFixedTendonConstraint&) = default;

  /* Computes the constraint Jacobian, independent of the configuration for this
   constraint. */
  static SapConstraintJacobian<T> CalcConstraintJacobian(
      const Parameters& parameters, const Kinematics& kinematics);

  /* Implementations of SapConstraint NVI functions. */
  std::unique_ptr<AbstractValue> DoMakeData(
      const T& dt,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const override;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& v,
                  AbstractValue* abstract_data) const override;
  T DoCalcCost(const AbstractValue& abstract_data) const override;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const override;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const override;
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapFixedTendonConstraint<T>>(
        new SapFixedTendonConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;
  void DoAccumulateGeneralizedImpulses(
      int c, const Eigen::Ref<const VectorX<T>>& gamma,
      EigenPtr<VectorX<T>> tau) const final;
  // no-op for this constraint.
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final {};

  VectorX<T> g_;  // Constraint function g. See CalcConstraintFunction().
  Parameters parameters_;
  Kinematics kinematics_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
