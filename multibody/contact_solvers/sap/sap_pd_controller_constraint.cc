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

  // Estimate regularization based on near-rigid regime threshold.
  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor = beta * beta / (4.0 * M_PI * M_PI);

  // Effective gain values are clamped in the near-rigid regime.
  T Kp_eff = parameters_.Kp();
  T Kd_eff = parameters_.Kd();

  // "Effective regularization" [Castro et al., 2021] for this constraint.
  const T R = 1.0 / (dt * (dt * Kp_eff + Kd_eff));

  // "Near-rigid" regularization, [Castro et al., 2021].
  const T& w = delassus_estimation[0];
  const T R_near_rigid = beta_factor * w;

  // In the near rigid regime we clamp Kp and Kd so that the effective
  // regularization is Rnr.
  if (R < R_near_rigid) {
    // Per [Castro et al., 2021], the relaxation time tau
    // for a critically damped constraint equals the time step, tau = dt.
    // With Kd = tau * Kp, and R = Rₙᵣ, we obtain Rₙᵣ⁻¹ = 2δt² Kₚ.
    Kp_eff = 1.0 / R_near_rigid / (2.0 * dt * dt);
    Kd_eff = dt * Kp_eff;
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
