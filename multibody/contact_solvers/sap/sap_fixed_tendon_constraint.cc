#include "drake/multibody/contact_solvers/sap/sap_fixed_tendon_constraint.h"

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
SapFixedTendonConstraint<T>::Parameters::Parameters(const T& lower_limit,
                                                    const T& upper_limit,
                                                    const T& stiffness,
                                                    const T& damping,
                                                    double beta)
    : lower_limit_(lower_limit),
      upper_limit_(upper_limit),
      stiffness_(stiffness),
      damping_(damping),
      beta_(beta) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  DRAKE_DEMAND(lower_limit < kInf);
  DRAKE_DEMAND(upper_limit > -kInf);
  DRAKE_DEMAND(lower_limit <= upper_limit);
  DRAKE_DEMAND(stiffness > 0);
  DRAKE_DEMAND(damping >= 0);
}

template <typename T>
SapFixedTendonConstraint<T>::Kinematics::Kinematics(
    int clique0, int clique1, int clique0_nv, int clique1_nv, VectorX<T> q0,
    VectorX<T> q1, VectorX<T> a0, VectorX<T> a1, T offset)
    : clique0_(clique0),
      clique1_(clique1),
      clique0_nv_(clique0_nv),
      clique1_nv_(clique1_nv),
      q0_(std::move(q0)),
      q1_(std::move(q1)),
      a0_(std::move(a0)),
      a1_(std::move(a1)),
      offset_(std::move(offset)) {
  DRAKE_DEMAND(clique0_ >= 0);
  DRAKE_DEMAND(clique1_ >= -1);
  DRAKE_DEMAND(clique0_ != clique1_);
  DRAKE_DEMAND(clique0_nv_ >= 0);
  DRAKE_DEMAND(q0_.size() == clique0_nv_);
  DRAKE_DEMAND(a0_.size() == clique0_nv_);
  if (clique1_ >= 0) {
    DRAKE_DEMAND(clique1_nv_ >= 0);
    DRAKE_DEMAND(q1_.size() == clique1_nv_);
    DRAKE_DEMAND(a1_.size() == clique1_nv_);
  }
}

template <typename T>
SapFixedTendonConstraint<T>::Kinematics::Kinematics(int clique0, int clique0_nv,
                                                    VectorX<T> q0,
                                                    VectorX<T> a0, T offset)
    : clique0_(clique0),
      clique0_nv_(clique0_nv),
      q0_(std::move(q0)),
      a0_(std::move(a0)),
      offset_(std::move(offset)) {
  DRAKE_DEMAND(clique0_ >= 0);
  DRAKE_DEMAND(clique0_ != clique1_);
  DRAKE_DEMAND(clique0_nv_ >= 0);
  DRAKE_DEMAND(q0_.size() == clique0_nv_);
  DRAKE_DEMAND(a0_.size() == clique0_nv_);
}

template <typename T>
SapFixedTendonConstraint<T>::SapFixedTendonConstraint(Parameters parameters,
                                                      Kinematics kinematics)
    : SapConstraint<T>(CalcConstraintJacobian(parameters, kinematics), {}),
      g_(CalcConstraintFunction(parameters, kinematics)),
      parameters_(std::move(parameters)),
      kinematics_(std::move(kinematics)) {}

template <typename T>
VectorX<T> SapFixedTendonConstraint<T>::CalcConstraintFunction(
    const Parameters& parameters, const Kinematics& kinematics) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const T& ll = parameters.lower_limit();
  const T& ul = parameters.upper_limit();

  const int nk = ll > -kInf && ul < kInf ? 2 : 1;
  VectorX<T> g0(nk);

  int i = 0;
  if (ll > -kInf) {
    g0(i) = kinematics.a0().dot(kinematics.q0()) + kinematics.offset() - ll;
    if (kinematics.clique1() >= 0) {
      g0(i) += kinematics.a1().dot(kinematics.q1());
    }
    ++i;
  }
  if (ul < kInf) {
    g0(i) = ul - kinematics.a0().dot(kinematics.q0()) - kinematics.offset();
    if (kinematics.clique1() >= 0) {
      g0(i) -= kinematics.a1().dot(kinematics.q1());
    }
  }

  return g0;
}

template <typename T>
SapConstraintJacobian<T> SapFixedTendonConstraint<T>::CalcConstraintJacobian(
    const Parameters& parameters, const Kinematics& kinematics) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const T& ll = parameters.lower_limit();
  const T& ul = parameters.upper_limit();

  const int nk = ll > -kInf && ul < kInf ? 2 : 1;
  MatrixX<T> J0 = MatrixX<T>::Zero(nk, kinematics.clique0_nv());

  int i = 0;
  if (ll > -kInf) J0.row(i++) += kinematics.a0();
  if (ul < kInf) J0.row(i) -= kinematics.a0();

  if (kinematics.clique1() >= 0) {
    MatrixX<T> J1 = MatrixX<T>::Zero(nk, kinematics.clique1_nv());

    i = 0;
    if (ll > -kInf) J1.row(i++) += kinematics.a1();
    if (ul < kInf) J1.row(i) -= kinematics.a1();

    return SapConstraintJacobian<T>(kinematics.clique0(), std::move(J0),
                                    kinematics.clique1(), std::move(J1));
  } else {
    return SapConstraintJacobian<T>(kinematics.clique0(), std::move(J0));
  }
}

template <typename T>
std::unique_ptr<AbstractValue> SapFixedTendonConstraint<T>::DoMakeData(
    const T& dt,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  // Estimate regularization based on near-rigid regime threshold.
  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2022] for details.
  const double beta_factor =
      parameters_.beta() * parameters_.beta() / (4.0 * M_PI * M_PI);

  T k_eff = parameters_.stiffness();
  T taud_eff = parameters_.damping() / k_eff;

  // "Effective regularization" [Castro et al., 2022] for this constraint.
  const T R_eff = 1.0 / (dt * k_eff * (dt + taud_eff));

  // "Near-rigid" regularization, [Castro et al., 2021].
  VectorX<T> R = (beta_factor * delassus_estimation).cwiseMax(R_eff);

  // Make data.
  SapFixedTendonConstraintData<T> data;
  typename SapFixedTendonConstraintData<T>::InvariantData& p =
      data.invariant_data;
  p.H = R.cwiseInverse();

  // If the effective relaxation R_eff is smaller than the near-rigid regime
  // relaxation R_near_rigid, that means that time_step will not be able to
  // resolve the dynamics introduced by this constraint. We call this the
  // "near-rigid" regime. Here we clamp taud to the time step, leading to a
  // critically damped constraint. Thus if this constraint is in the
  // "near-rigid" regime, v̂ = -g₀ / 2⋅δt.
  //
  // Refer to Section V of [Castro et al., 2022] for further details.
  p.v_hat = -g_;
  for (int i = 0; i < this->num_constraint_equations(); ++i) {
    if (R_eff < R(i)) {
      p.v_hat(i) /= 2 * dt;
    } else {
      p.v_hat(i) /= (dt + taud_eff);
    }
  }

  return AbstractValue::Make(data);
}

template <typename T>
void SapFixedTendonConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>& v, AbstractValue* abstract_data) const {
  auto& data =
      abstract_data->get_mutable_value<SapFixedTendonConstraintData<T>>();

  const VectorX<T>& v_hat = data.invariant_data.v_hat;
  const VectorX<T>& H = data.invariant_data.H;

  // This constraint is formulated such that the cost, impulse, and hessian
  // are all zero when the constraint is not active.
  data.v_ = v;
  data.hessian_ = VectorX<T>::Zero(this->num_constraint_equations());
  data.impulse_ = VectorX<T>::Zero(this->num_constraint_equations());
  data.cost_ = T(0);

  for (int i = 0; i < this->num_constraint_equations(); ++i) {
    // Constraint is active when v < v̂
    if (v(i) < v_hat(i)) {
      const T dv = v_hat(i) - v(i);
      data.hessian_(i) = H(i);
      data.impulse_(i) = H(i) * dv;
      data.cost_ += 0.5 * H(i) * dv * dv;
    }
  }
}

template <typename T>
T SapFixedTendonConstraint<T>::DoCalcCost(
    const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapFixedTendonConstraintData<T>>();
  return data.cost_;
}

template <typename T>
void SapFixedTendonConstraint<T>::DoCalcImpulse(
    const AbstractValue& abstract_data, EigenPtr<VectorX<T>> gamma) const {
  const auto& data = abstract_data.get_value<SapFixedTendonConstraintData<T>>();
  *gamma = data.impulse_;
}

template <typename T>
void SapFixedTendonConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data = abstract_data.get_value<SapFixedTendonConstraintData<T>>();
  *G = data.hessian_.asDiagonal();
}

template <typename T>
void SapFixedTendonConstraint<T>::DoAccumulateGeneralizedImpulses(
    int c, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  // For this constraint the generalized impulses are simply τ = Jᵀ⋅γ.
  if (c == 0) {
    this->first_clique_jacobian().TransposeAndMultiplyAndAddTo(gamma, tau);
  } else if (c == 1) {
    this->second_clique_jacobian().TransposeAndMultiplyAndAddTo(gamma, tau);
  } else {
    DRAKE_UNREACHABLE();
  }
}

template <typename T>
std::unique_ptr<SapConstraint<double>> SapFixedTendonConstraint<T>::DoToDouble()
    const {
  const typename SapFixedTendonConstraint<T>::Parameters& p = parameters_;
  const typename SapFixedTendonConstraint<T>::Kinematics& k = kinematics_;

  SapFixedTendonConstraint<double>::Parameters p_to_double(
      ExtractDoubleOrThrow(p.lower_limit()),
      ExtractDoubleOrThrow(p.upper_limit()),
      ExtractDoubleOrThrow(p.stiffness()), ExtractDoubleOrThrow(p.damping()),
      p.beta());

  SapFixedTendonConstraint<double>::Kinematics k_to_double(
      k.clique0(), k.clique1(), k.clique0_nv(), k.clique1_nv(),
      math::DiscardGradient(k.q0()), math::DiscardGradient(k.q1()),
      math::DiscardGradient(k.a0()), math::DiscardGradient(k.a1()),
      ExtractDoubleOrThrow(k.offset()));

  return std::make_unique<SapFixedTendonConstraint<double>>(
      std::move(p_to_double), std::move(k_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapFixedTendonConstraint);
