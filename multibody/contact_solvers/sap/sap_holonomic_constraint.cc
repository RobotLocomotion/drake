#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapHolonomicConstraint<T>::Parameters::Parameters(
    VectorX<T> impulse_lower_limits, VectorX<T> impulse_upper_limits,
    VectorX<T> stiffnesses, VectorX<T> relaxation_times, double beta)
    : impulse_lower_limits_(std::move(impulse_lower_limits)),
      impulse_upper_limits_(std::move(impulse_upper_limits)),
      stiffnesses_(std::move(stiffnesses)),
      relaxation_times_(std::move(relaxation_times)),
      beta_(beta) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  DRAKE_DEMAND(impulse_lower_limits_.size() == impulse_upper_limits_.size());
  DRAKE_DEMAND(impulse_lower_limits_.size() == stiffnesses_.size());
  DRAKE_DEMAND(impulse_lower_limits_.size() == relaxation_times_.size());
  DRAKE_DEMAND((impulse_lower_limits_.array() <= kInf).all());
  DRAKE_DEMAND((impulse_upper_limits_.array() >= -kInf).all());
  DRAKE_DEMAND(
      (impulse_lower_limits_.array() <= impulse_upper_limits_.array()).all());
  DRAKE_DEMAND((stiffnesses_.array() > 0).all());
  DRAKE_DEMAND((relaxation_times_.array() >= 0).all());
}

template <typename T>
SapHolonomicConstraint<T>::SapHolonomicConstraint(Kinematics kinematics,
                                                  Parameters parameters,
                                                  std::vector<int> objects)
    : SapConstraint<T>(std::move(kinematics.J), std::move(objects)),
      g_(std::move(kinematics.g)),
      bias_(std::move(kinematics.b)),
      parameters_(std::move(parameters)) {}

template <typename T>
SapHolonomicConstraint<T>::SapHolonomicConstraint(VectorX<T> g,
                                                  SapConstraintJacobian<T> J,
                                                  Parameters parameters)
    : SapConstraint<T>(std::move(J), {}),
      g_(std::move(g)),
      parameters_(std::move(parameters)) {
  DRAKE_THROW_UNLESS(g_.size() == this->jacobian().rows());
  DRAKE_THROW_UNLESS(g_.size() == parameters_.num_constraint_equations());
  bias_.setZero(this->num_constraint_equations());
}

template <typename T>
SapHolonomicConstraint<T>::SapHolonomicConstraint(VectorX<T> g,
                                                  SapConstraintJacobian<T> J,
                                                  VectorX<T> b,
                                                  Parameters parameters)
    : SapConstraint<T>(std::move(J), {}),
      g_(std::move(g)),
      bias_(std::move(b)),
      parameters_(std::move(parameters)) {
  DRAKE_THROW_UNLESS(g_.size() == this->jacobian().rows());
  DRAKE_THROW_UNLESS(bias_.size() == this->jacobian().rows());
  DRAKE_THROW_UNLESS(g_.size() == parameters_.num_constraint_equations());
}

template <typename T>
std::unique_ptr<AbstractValue> SapHolonomicConstraint<T>::DoMakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  const double beta = parameters_.beta();

  // Estimate regularization based on near-rigid regime threshold.
  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor = beta * beta / (4.0 * M_PI * M_PI);

  // Relaxation used in the near-rigid regime.
  const VectorX<T> R_near_rigid = beta_factor * delassus_estimation;

  // Regularization for the specified stiffness and dissipation.
  const VectorX<T>& k = parameters_.stiffnesses();
  VectorX<T> tau = parameters_.relaxation_times();
  const VectorX<T> R_compliant =
      1.0 / (time_step * (time_step + tau.array()) * k.array());

  VectorX<T> R = R_compliant.array().max(R_near_rigid.array());

  // Adjusted relaxation times to the near-rigid regime.
  // In this near-rigid regime a relaxation time equal to the time step leads to
  // a critically damped constraint, [Castro et al., 2022].
  for (int e = 0; e < this->num_constraint_equations(); ++e) {
    // If the relaxation R is smaller than the near-rigid regime relaxation
    // R_near_rigid, that means that time_step will not be able to resolve the
    // dynamics introduced by this constraint. We call this the "near-rigid"
    // regime. Here we clamp relaxation time to the time step. Refer to Section
    // V of [Castro et al., 2022] for further details.
    if (R_compliant(e) < R_near_rigid(e)) {
      tau(e) = time_step;
    }
  }

  // Componentwise computation: v̂ᵢ = −gᵢ / (δt + τᵢ) − bᵢ.
  VectorX<T> v_hat = -VectorX<T>(this->constraint_function().array() /
                                 (time_step + tau.array())) -
                     bias_;

  // Make data.
  SapHolonomicConstraintData<T> data(std::move(R), std::move(v_hat));
  return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
}

template <typename T>
void SapHolonomicConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>& vc,
    AbstractValue* abstract_data) const {
  auto& data =
      abstract_data->get_mutable_value<SapHolonomicConstraintData<T>>();
  const VectorX<T>& gl = parameters_.impulse_lower_limits();
  const VectorX<T>& gu = parameters_.impulse_upper_limits();
  const VectorX<T>& R_inv = data.R_inv();
  const VectorX<T>& v_hat = data.v_hat();
  VectorX<T>& y = data.mutable_y();

  data.mutable_vc() = vc;
  y = R_inv.asDiagonal() * (v_hat - vc);

  // The analytical projection γ = P(y) results in bounds componentwise, i.e.
  //   γ = max(gₗ, min(gᵤ, y)).
  data.mutable_gamma() = y.array().max(gl.array()).min(gu.array());
}

template <typename T>
T SapHolonomicConstraint<T>::DoCalcCost(
    const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapHolonomicConstraintData<T>>();
  const int nk = this->num_constraint_equations();
  const VectorX<T>& gl = parameters_.impulse_lower_limits();
  const VectorX<T>& gu = parameters_.impulse_upper_limits();
  const VectorX<T>& R = data.R();
  const VectorX<T>& vc = data.vc();
  const VectorX<T>& y = data.y();

  T cost = 0.0;
  for (int i = 0; i < nk; ++i) {
    if (y(i) < gl(i)) {  // Below lower limit.
      cost -= gl(i) * vc(i);
    } else if (y(i) > gu(i)) {  // Above upper limit.
      cost -= gu(i) * vc(i);
    } else {
      cost += 0.5 * R(i) * y(i) * y(i);
    }
  }
  return cost;
}

template <typename T>
void SapHolonomicConstraint<T>::DoCalcImpulse(
    const AbstractValue& abstract_data, EigenPtr<VectorX<T>> gamma) const {
  const auto& data = abstract_data.get_value<SapHolonomicConstraintData<T>>();
  *gamma = data.gamma();
}

template <typename T>
void SapHolonomicConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data = abstract_data.get_value<SapHolonomicConstraintData<T>>();
  const VectorX<T>& gl = parameters_.impulse_lower_limits();
  const VectorX<T>& gu = parameters_.impulse_upper_limits();
  const VectorX<T>& R_inv = data.R_inv();
  const VectorX<T>& y = data.y();

  const int nk = this->num_constraint_equations();
  (*G) = MatrixX<T>::Zero(nk, nk);
  for (int i = 0; i < nk; ++i) {
    if (gl(i) < y(i) && y(i) < gu(i)) {
      (*G)(i, i) = R_inv(i);
    }
  }
}

template <typename T>
std::unique_ptr<SapConstraint<double>> SapHolonomicConstraint<T>::DoToDouble()
    const {
  const typename SapHolonomicConstraint<T>::Parameters& p = parameters_;
  SapHolonomicConstraint<double>::Parameters p_to_double(
      math::DiscardGradient(p.impulse_lower_limits()),
      math::DiscardGradient(p.impulse_upper_limits()),
      math::DiscardGradient(p.stiffnesses()),
      math::DiscardGradient(p.relaxation_times()), p.beta());
  return std::make_unique<SapHolonomicConstraint<double>>(
      math::DiscardGradient(constraint_function()), this->jacobian().ToDouble(),
      math::DiscardGradient(bias()), std::move(p_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapHolonomicConstraint);
