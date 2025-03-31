#include "drake/multibody/contact_solvers/sap/sap_tendon_constraint.h"

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
bool SapTendonConstraint<T>::Parameters::has_finite_lower_limit() const {
  return lower_limit_ > -std::numeric_limits<double>::infinity();
}

template <typename T>
bool SapTendonConstraint<T>::Parameters::has_finite_upper_limit() const {
  return upper_limit_ < std::numeric_limits<double>::infinity();
}

template <typename T>
SapTendonConstraint<T>::Parameters::Parameters(const T& lower_limit,
                                               const T& upper_limit,
                                               const T& stiffness,
                                               const T& damping, double beta)
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
  num_finite_limits_ =
      has_finite_lower_limit() && has_finite_upper_limit() ? 2 : 1;
}

template <typename T>
SapTendonConstraint<T>::Kinematics::Kinematics(int clique0, int clique1,
                                               VectorX<T> q0, VectorX<T> q1,
                                               VectorX<T> a0, VectorX<T> a1,
                                               T offset)
    : clique0_(clique0),
      clique1_(clique1),
      q0_(std::move(q0)),
      q1_(std::move(q1)),
      a0_(std::move(a0)),
      a1_(std::move(a1)),
      offset_(std::move(offset)) {
  DRAKE_DEMAND(clique0_ >= 0);
  DRAKE_DEMAND(*clique1_ >= 0);
  DRAKE_DEMAND(clique0_ != *clique1_);
  DRAKE_DEMAND(q0_.size() == a0_.size());
  DRAKE_DEMAND(q1_.size() == a1_.size());
}

template <typename T>
SapTendonConstraint<T>::Kinematics::Kinematics(int clique0, VectorX<T> q0,
                                               VectorX<T> a0, T offset)
    : clique0_(clique0),
      q0_(std::move(q0)),
      a0_(std::move(a0)),
      offset_(std::move(offset)) {
  DRAKE_DEMAND(clique0_ >= 0);
  DRAKE_DEMAND(q0_.size() == a0_.size());
}

template <typename T>
SapTendonConstraint<T>::SapTendonConstraint(Parameters parameters,
                                            Kinematics kinematics)
    : SapConstraint<T>(CalcConstraintJacobian(parameters, kinematics), {}),
      g_(CalcConstraintFunction(parameters, kinematics)),
      parameters_(std::move(parameters)),
      kinematics_(std::move(kinematics)) {}

template <typename T>
VectorX<T> SapTendonConstraint<T>::CalcConstraintFunction(
    const Parameters& parameters, const Kinematics& kinematics) {
  const T& ll = parameters.lower_limit();
  const T& ul = parameters.upper_limit();

  VectorX<T> g0(parameters.num_finite_limits());

  T l0 = kinematics.a0().dot(kinematics.q0()) + kinematics.offset();
  if (kinematics.num_cliques() > 1) {
    l0 += kinematics.a1().dot(kinematics.q1());
  }

  if (parameters.has_finite_lower_limit()) {
    g0.head(1)(0) = l0 - ll;
  }
  if (parameters.has_finite_upper_limit()) {
    g0.tail(1)(0) = ul - l0;
  }

  return g0;
}

template <typename T>
SapConstraintJacobian<T> SapTendonConstraint<T>::CalcConstraintJacobian(
    const Parameters& parameters, const Kinematics& kinematics) {
  MatrixX<T> J0 =
      MatrixX<T>::Zero(parameters.num_finite_limits(), kinematics.clique0_nv());

  if (parameters.has_finite_lower_limit()) {
    J0.topRows(1) += kinematics.a0().transpose();
  }
  if (parameters.has_finite_upper_limit()) {
    J0.bottomRows(1) -= kinematics.a0().transpose();
  }

  if (kinematics.num_cliques() > 1) {
    MatrixX<T> J1 = MatrixX<T>::Zero(parameters.num_finite_limits(),
                                     kinematics.clique1_nv());

    if (parameters.has_finite_lower_limit()) {
      J1.topRows(1) += kinematics.a1().transpose();
    }
    if (parameters.has_finite_upper_limit()) {
      J1.bottomRows(1) -= kinematics.a1().transpose();
    }
    return SapConstraintJacobian<T>(kinematics.clique0(), std::move(J0),
                                    kinematics.clique1(), std::move(J1));
  } else {
    return SapConstraintJacobian<T>(kinematics.clique0(), std::move(J0));
  }
}

template <typename T>
std::unique_ptr<AbstractValue> SapTendonConstraint<T>::DoMakeData(
    const T& dt,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  // Estimate regularization based on near-rigid regime threshold.
  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2022] for details.
  const double beta_factor =
      parameters_.beta() * parameters_.beta() / (4.0 * M_PI * M_PI);

  const T k = parameters_.stiffness();
  const T taud = parameters_.damping() / k;

  // Compliant regularization.
  const T R_compliant = 1.0 / (dt * k * (dt + taud));

  // "Near-rigid" regularization, [Castro et al., 2021].
  const VectorX<T> R_near_rigid = beta_factor * delassus_estimation;

  // Effective regularization.
  const VectorX<T> R_effective = (R_near_rigid).cwiseMax(R_compliant);

  // Make data.
  SapTendonConstraintData<T> data;
  typename SapTendonConstraintData<T>::InvariantData& p = data.invariant_data;
  p.H = R_effective.cwiseInverse();

  // If the effective relaxation R_compliant is smaller than the near-rigid
  // regime relaxation R_near_rigid, that means that time_step will not be able
  // to resolve the dynamics introduced by this constraint. We call this the
  // "near-rigid" regime. Here we clamp taud to the time step, leading to a
  // critically damped constraint. Thus if this constraint is in the
  // "near-rigid" regime, v̂ = -g₀ / 2⋅δt.
  //
  // Refer to Section V of [Castro et al., 2022] for further details.
  p.v_hat = -g_;
  for (int i = 0; i < this->num_constraint_equations(); ++i) {
    if (R_compliant < R_near_rigid(i)) {
      p.v_hat(i) /= (2 * dt);
    } else {
      p.v_hat(i) /= (dt + taud);
    }
  }

  return AbstractValue::Make(data);
}

template <typename T>
void SapTendonConstraint<T>::DoCalcData(const Eigen::Ref<const VectorX<T>>& v,
                                        AbstractValue* abstract_data) const {
  auto& data = abstract_data->get_mutable_value<SapTendonConstraintData<T>>();

  const VectorX<T>& v_hat = data.invariant_data.v_hat;
  const VectorX<T>& H = data.invariant_data.H;

  // This constraint is formulated such that the cost, impulse, and hessian
  // are all zero when the constraint is not active.
  data.v = v;
  data.hessian = VectorX<T>::Zero(this->num_constraint_equations());
  data.impulse = VectorX<T>::Zero(this->num_constraint_equations());
  data.cost = T(0);

  for (int i = 0; i < this->num_constraint_equations(); ++i) {
    // Constraint is active when v < v̂.
    if (v(i) < v_hat(i)) {
      const T dv = v_hat(i) - v(i);
      data.hessian(i) = H(i);
      data.impulse(i) = H(i) * dv;
      data.cost += 0.5 * H(i) * dv * dv;
    }
  }
}

template <typename T>
T SapTendonConstraint<T>::DoCalcCost(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapTendonConstraintData<T>>();
  return data.cost;
}

template <typename T>
void SapTendonConstraint<T>::DoCalcImpulse(const AbstractValue& abstract_data,
                                           EigenPtr<VectorX<T>> gamma) const {
  const auto& data = abstract_data.get_value<SapTendonConstraintData<T>>();
  *gamma = data.impulse;
}

template <typename T>
void SapTendonConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data = abstract_data.get_value<SapTendonConstraintData<T>>();
  *G = data.hessian.asDiagonal();
}

template <typename T>
void SapTendonConstraint<T>::DoAccumulateGeneralizedImpulses(
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
void SapTendonConstraint<T>::DoAccumulateSpatialImpulses(
    int, const Eigen::Ref<const VectorX<T>>&, SpatialForce<T>*) const {}

template <typename T>
std::unique_ptr<SapConstraint<T>> SapTendonConstraint<T>::DoClone() const {
  return std::unique_ptr<SapTendonConstraint<T>>(
      new SapTendonConstraint<T>(*this));
}

template <typename T>
std::unique_ptr<SapConstraint<double>> SapTendonConstraint<T>::DoToDouble()
    const {
  const typename SapTendonConstraint<T>::Parameters& p = parameters_;
  const typename SapTendonConstraint<T>::Kinematics& k = kinematics_;

  SapTendonConstraint<double>::Parameters p_to_double(
      ExtractDoubleOrThrow(p.lower_limit()),
      ExtractDoubleOrThrow(p.upper_limit()),
      ExtractDoubleOrThrow(p.stiffness()), ExtractDoubleOrThrow(p.damping()),
      p.beta());

  if (k.num_cliques() > 1) {
    SapTendonConstraint<double>::Kinematics k_to_double(
        k.clique0(), k.clique1(), math::DiscardGradient(k.q0()),
        math::DiscardGradient(k.q1()), math::DiscardGradient(k.a0()),
        math::DiscardGradient(k.a1()), ExtractDoubleOrThrow(k.offset()));

    return std::make_unique<SapTendonConstraint<double>>(
        std::move(p_to_double), std::move(k_to_double));
  } else {
    SapTendonConstraint<double>::Kinematics k_to_double(
        k.clique0(), math::DiscardGradient(k.q0()),
        math::DiscardGradient(k.a0()), ExtractDoubleOrThrow(k.offset()));

    return std::make_unique<SapTendonConstraint<double>>(
        std::move(p_to_double), std::move(k_to_double));
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapTendonConstraint);
