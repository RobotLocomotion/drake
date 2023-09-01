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
  DRAKE_DEMAND(Kp_ >= 0.0);
  DRAKE_DEMAND(Kd_ >= 0.0);
  DRAKE_DEMAND(effort_limit_ > 0.0);
}

template <typename T>
SapPdControllerConstraint<T>::SapPdControllerConstraint(
    Configuration configuration, Parameters parameters)
    : SapConstraint<T>(MakeConstraintJacobian(configuration), {}),
      configuration_(std::move(configuration)),
      parameters_(std::move(parameters)) {}

template <typename T>
std::unique_ptr<AbstractValue> SapPdControllerConstraint<T>::DoMakeData(
    const T& dt,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  // TODO(amcastrot-tri): consider exposing the near rigid parameter.
  constexpr double beta = 0.1;

  // Estimate regularization based on near-rigid regime threshold.
  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor = beta * beta / (4.0 * M_PI * M_PI);

  T Kp = parameters_.Kp();
  T Kd = parameters_.Kd();

  // "Effective regularization" [Castro et al., 2021] for this constraint.
  const T R = 1.0 / (dt * (dt * Kp + Kd));

  // "Near-rigid" regularization, [Castro et al., 2021].
  const T& w = delassus_estimation[0];
  const T Rnr = beta_factor * w;

  // In the near rigid regime we clamp Kp and Kd so that the effective
  // regularization is Rnr.
  if (R < Rnr) {
    // Per [Castro et al., 2021], the relaxation time tau
    // for a critically damped constraint equals the time step, tau = dt.
    // With Kd = tau * Kp, and R = Rₙᵣ, we obtain Rₙᵣ⁻¹ = 2δt² Kₚ.
    Kp = 1.0 / Rnr / (2.0 * dt * dt);
    Kd = dt * Kp;
  }

  // Make data.
  SapPdControllerConstraintData<T> data(Kp, Kd, dt);
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
  const T& Kp = data.Kp();
  const T& Kd = data.Kd();
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
  const T Kd_discrete = dt * Kp + Kd;
  data.mutable_v() = v;
  data.mutable_cost() = dt * ClampAntiDerivative(y, e) / Kd_discrete;
  data.mutable_impulse() = dt * Clamp(y, e);
  data.mutable_hessian() = dt * Kd_discrete * ClampDerivative(y, e);
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

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapPdControllerConstraint)
