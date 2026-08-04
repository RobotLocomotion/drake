#include "drake/multibody/contact_solvers/sap/sap_pd_controller_constraint.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapPdControllerConstraint<T>::Parameters::Parameters(T Kp, T Kd, T effort_limit)
    : Kp_(std::move(Kp)),
      Kd_(std::move(Kd)),
      effort_limit_(std::move(effort_limit)) {
  using std::isfinite;
  using std::isnan;
  DRAKE_DEMAND(isfinite(Kp_));
  DRAKE_DEMAND(isfinite(Kd_));
  DRAKE_DEMAND(!isnan(effort_limit_));
  DRAKE_DEMAND(Kp_ >= 0.0);
  DRAKE_DEMAND(Kd_ >= 0.0);
  DRAKE_DEMAND(Kp_ > 0.0 || Kd_ > 0.0);
  DRAKE_DEMAND(effort_limit_ > 0.0);
}

template <typename T>
SapPdControllerConstraint<T>::SapPdControllerConstraint(
    Configuration configuration, Parameters parameters)
    : SapConstraint<T>(MakeConstraintJacobian(configuration), {}),
      configuration_(std::move(configuration)),
      parameters_(std::move(parameters)) {}

/* N.B. This method makes a SapPdControllerConstraintData with "effective"
 values of PD gains that might differ from the user provided values. This
 happens in the "near-rigid regime" (see below), when the simulation time-step
 cannot resolve the fast dynamics introduced by the model of the controller.
 Since in this case the implicit scheme will unavoidably dampen the dynamics,
 the solver decides to clamp stiff values to condition the problem numerics
 better, without sacrificing accuracy (since the given time step size cannot
 resolve this fast dynamics anyway). When the specified time step size can
 resolve the dynamics of the controller, these effective values will equal the
 user provided Kp and Kd values. */
template <typename T>
std::unique_ptr<AbstractValue> SapPdControllerConstraint<T>::DoMakeData(
    const T& dt,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  // TODO(amcastro-tri): consider exposing the near rigid parameter.
  constexpr double beta = 0.1;

  // In the near-rigid regime [Castro et al., 2021], the constraint time scale
  // is limited to Tₙᵣ = β⋅δt, with β < 1,  making it under-resolved. This sets
  // a lower bound on the regularization parameter: Rₙᵣ = β²/(4π²)⋅wᵢ, where wᵢ
  // is the Delassus estimate, ensuring controlled numerical conditioning.
  const double beta_factor = beta * beta / (4.0 * M_PI * M_PI);
  const T R_nr = beta_factor * delassus_estimation[0];

  // "Effective regularization" R [Castro et al., 2021] for this constraint
  // based on user specified gains:
  const T& Kp = parameters_.Kp();
  const T& Kd = parameters_.Kd();
  const T R = 1.0 / (dt * (dt * Kp + Kd));

  // To keep numerical conditioning under control, [Castro et al., 2021] propose
  // to use Rₙᵣ as a lower bound on R such that constraint time scale is
  // Tₙᵣ = β⋅δt, i.e. under resolved for β < 1.
  //
  // However, we also want to respect the ratio Kd/Kp set by the user. In
  // particular, if Kp = 0, that means the user wants velocity control only.
  // Similarly, if Kd = 0, the user wants position control only. From the
  // expression above for R, we observe we can write the effective
  // regularization R in two different but equivalent ways:
  //  R = 1/(δt⋅(δt +   Kd/Kp )⋅Kp),   if Kp > 0
  //  R = 1/(δt⋅( 1 + δt⋅Kp/Kd)⋅Kd),   if Kd > 0
  // Keeping the ratio τ = Kd/Kp (or its inverse) constant, and equating R =
  // Rₙᵣ, we can obtain expressions for the near-rigid regime gains from:
  //  Rₙᵣ = 1/(δt⋅(δt +   Kd/Kp )⋅Kpₙᵣ),   if Kp > 0
  //  Rₙᵣ = 1/(δt⋅( 1 + δt⋅Kp/Kd)⋅Kdₙᵣ),   if Kd > 0
  // And solving for Kpₙᵣ and Kdₙᵣ as:
  //  1) Kpₙᵣ = 1/(δt⋅(δt +   Kd/Kp )⋅Rₙᵣ),   if Kp > 0
  //  2) Kdₙᵣ = 1/(δt⋅( 1 + δt⋅Kp/Kd)⋅Rₙᵣ),   if Kd > 0
  //
  // It is clear that when Kp = 0 we should use (2) and when Kd = 0 we should
  // use (1). However, we'd also like to be careful for when either Kp or Kd are
  // close to zero to avoid round-off errors. We could use a fixed threshold,
  // but instead we observe we can have a smooth transition between (1) and (2)
  // by analyzing their dimensionless forms:
  //  1*) Kp* = δt⋅Kpₙᵣ⋅(δt⋅Rₙᵣ) =  1/(1 + τ*)
  //  2*) Kd* =    Kdₙᵣ⋅(δt⋅Rₙᵣ) = τ*/(1 + τ*)
  // with τ* = τ/δt the dimensionless dissipation time constant. In this
  // dimensionless form, Kp* + Kd* = 1, Kp*/Kd* = τ*, and thus τ* controls the
  // relative contribution of Kp and Kd to the effective regularization.
  //
  // Therefore, to minimize round-off errors, the criterion we use to choose
  // from (1*) or (2*) is to use the expression that makes Kp* (or Kd*) farthest
  // from zero. That is, we use (1*) whenever τ* < 1 and (2*) when τ* ≥ 1. This
  // criterion keeps Kp* (or Kd*) in the range (1/2; 1) at all times.
  //
  // Back to the dimensional form, and to avoid division by zero, we obtain our
  // first gain using (1) (for Kp) when δt⋅Kp > Kd and (2) (for Kd) when
  // δt⋅Kp ≤ Kd. The second gain is obtained from the condition to respect the
  // user provided Kp to Kd ratio.

  // We start from the original gains.
  T Kp_eff = Kp;
  T Kd_eff = Kd;
  if (R < R_nr) {  // If in the near-rigid regime, we limit the effective gains.
    if (dt * Kp > Kd) {
      const T tau = Kd / Kp;
      Kp_eff = 1.0 / (dt * (dt + tau) * R_nr);  // Equation (1).
      Kd_eff = tau * Kp_eff;                    // We keep ratio tau constant.
    } else {
      const T tau_inv = Kp / Kd;
      Kd_eff = 1.0 / (dt * (1.0 + dt * tau_inv) * R_nr);  // Equation (2).
      Kp_eff = tau_inv * Kd_eff;  // We keep ratio tau_inv constant.
    }
  }

  // Make data.
  SapPdControllerConstraintData<T> data(Kp_eff, Kd_eff, dt);
  return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
}

template <typename T>
void SapPdControllerConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>& vc,
    AbstractValue* abstract_data) const {
  auto& data =
      abstract_data->get_mutable_value<SapPdControllerConstraintData<T>>();
  // Const data.
  const T& dt = data.time_step();
  const T& Kp = data.Kp_eff();
  const T& Kd = data.Kd_eff();
  const T& e = parameters().effort_limit();

  // Previous configuration.
  const T& q0 = configuration().q0;
  const T& u0 = configuration().u0;

  // Desired state.
  const T& qd = configuration().qd;
  const T& vd = configuration().vd;

  // Next state.
  const T& v = vc[0];
  const T q = q0 + dt * v;

  // Update data as a function of v.
  const T y = -Kp * (q - qd) - Kd * (v - vd) + u0;
  const T common_factor = dt * Kp + Kd;
  data.mutable_v() = v;
  data.mutable_cost() = dt * ClampAntiderivative(y, e) / common_factor;
  data.mutable_impulse() = dt * Clamp(y, e);
  data.mutable_hessian() = dt * common_factor * ClampDerivative(y, e);
}

template <typename T>
T SapPdControllerConstraint<T>::DoCalcCost(
    const AbstractValue& abstract_data) const {
  const auto& data =
      abstract_data.get_value<SapPdControllerConstraintData<T>>();
  return data.cost();
}

template <typename T>
void SapPdControllerConstraint<T>::DoCalcImpulse(
    const AbstractValue& abstract_data, EigenPtr<VectorX<T>> gamma) const {
  const auto& data =
      abstract_data.get_value<SapPdControllerConstraintData<T>>();
  *gamma = Vector1<T>::Constant(data.impulse());
}

template <typename T>
void SapPdControllerConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data =
      abstract_data.get_value<SapPdControllerConstraintData<T>>();
  (*G)(0, 0) = data.hessian();
}

template <typename T>
void SapPdControllerConstraint<T>::DoAccumulateGeneralizedImpulses(
    int, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  (*tau)(configuration().clique_dof) += gamma(0);
}

template <typename T>
SapConstraintJacobian<T> SapPdControllerConstraint<T>::MakeConstraintJacobian(
    Configuration c) {
  MatrixX<T> J = RowVectorX<T>::Unit(c.clique_nv, c.clique_dof);
  return SapConstraintJacobian<T>(c.clique, std::move(J));
}

template <typename T>
std::unique_ptr<SapConstraint<double>>
SapPdControllerConstraint<T>::DoToDouble() const {
  const typename SapPdControllerConstraint<T>::Parameters& p = parameters_;
  const typename SapPdControllerConstraint<T>::Configuration& c =
      configuration_;
  SapPdControllerConstraint<double>::Parameters p_to_double(
      ExtractDoubleOrThrow(p.Kp()), ExtractDoubleOrThrow(p.Kd()),
      ExtractDoubleOrThrow(p.effort_limit()));
  SapPdControllerConstraint<double>::Configuration c_to_double{
      c.clique,
      c.clique_dof,
      c.clique_nv,
      ExtractDoubleOrThrow(c.q0),
      ExtractDoubleOrThrow(c.qd),
      ExtractDoubleOrThrow(c.vd),
      ExtractDoubleOrThrow(c.u0)};
  return std::make_unique<SapPdControllerConstraint<double>>(
      std::move(c_to_double), std::move(p_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapPdControllerConstraint);
