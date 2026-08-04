#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"

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
SapLimitConstraint<T>::Parameters::Parameters(const T& lower_limit,
                                              const T& upper_limit,
                                              const T& stiffness,
                                              const T& dissipation_time_scale,
                                              double beta)
    : lower_limit_(lower_limit),
      upper_limit_(upper_limit),
      stiffness_(stiffness),
      dissipation_time_scale_(dissipation_time_scale),
      beta_(beta) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  DRAKE_DEMAND(lower_limit < kInf);
  DRAKE_DEMAND(upper_limit > -kInf);
  DRAKE_DEMAND(lower_limit <= upper_limit);
  DRAKE_DEMAND(stiffness > 0);
  DRAKE_DEMAND(dissipation_time_scale > 0);
}

template <typename T>
SapLimitConstraint<T>::SapLimitConstraint(int clique, int clique_dof,
                                          int clique_nv, const T& q0,
                                          Parameters parameters)
    : SapConstraint<T>(CalcConstraintJacobian(clique, clique_dof, clique_nv,
                                              parameters.lower_limit(),
                                              parameters.upper_limit()),
                       {}),
      parameters_(std::move(parameters)),
      clique_dof_(clique_dof),
      q0_{q0},
      g_(CalcConstraintFunction(q0, parameters.lower_limit(),
                                parameters.upper_limit())) {}

template <typename T>
VectorX<T> SapLimitConstraint<T>::CalcConstraintFunction(const T& q0,
                                                         const T& ql,
                                                         const T& qu) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  DRAKE_DEMAND(ql < kInf);
  DRAKE_DEMAND(qu > -kInf);

  const int nk = ql > -kInf && qu < kInf ? 2 : 1;
  VectorX<T> g0(nk);

  int i = 0;
  if (ql > -kInf) g0(i++) = q0 - ql;  // lower limit.
  if (qu < kInf) g0(i) = qu - q0;     // upper limit.

  return g0;
}

template <typename T>
SapConstraintJacobian<T> SapLimitConstraint<T>::CalcConstraintJacobian(
    int clique, int clique_dof, int clique_nv, const T& ql, const T& qu) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  DRAKE_DEMAND(ql < kInf);
  DRAKE_DEMAND(qu > -kInf);
  DRAKE_DEMAND(ql <= qu);

  const int nk = ql > -kInf && qu < kInf ? 2 : 1;
  MatrixX<T> J = MatrixX<T>::Zero(nk, clique_nv);

  int i = 0;
  if (ql > -kInf) J(i++, clique_dof) = 1;
  if (qu < kInf) J(i, clique_dof) = -1;

  return SapConstraintJacobian<T>(clique, std::move(J));
}

template <typename T>
std::unique_ptr<AbstractValue> SapLimitConstraint<T>::DoMakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  using std::max;

  // Estimate regularization based on near-rigid regime threshold.
  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor =
      parameters_.beta() * parameters_.beta() / (4.0 * M_PI * M_PI);

  const T& k = parameters_.stiffness();
  const T& taud = parameters_.dissipation_time_scale();

  VectorX<T> R = (beta_factor * delassus_estimation)
                     .cwiseMax(1.0 / (time_step * k * (time_step + taud)));
  VectorX<T> v_hat = -g_ / (time_step + taud);

  // Make data.
  SapLimitConstraintData<T> data(std::move(R), std::move(v_hat));
  return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
}

template <typename T>
void SapLimitConstraint<T>::DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                                       AbstractValue* abstract_data) const {
  auto& data = abstract_data->get_mutable_value<SapLimitConstraintData<T>>();
  const VectorX<T>& R_inv = data.R_inv();
  const VectorX<T>& v_hat = data.v_hat();
  data.mutable_vc() = vc;
  data.mutable_y() = R_inv.asDiagonal() * (v_hat - vc);
  data.mutable_gamma() = data.y().array().max(T{0.0});
}

template <typename T>
T SapLimitConstraint<T>::DoCalcCost(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapLimitConstraintData<T>>();
  const VectorX<T>& R = data.R();
  const VectorX<T>& gamma = data.gamma();
  const T cost = 0.5 * gamma.dot(R.asDiagonal() * gamma);
  return cost;
}

template <typename T>
void SapLimitConstraint<T>::DoCalcImpulse(const AbstractValue& abstract_data,
                                          EigenPtr<VectorX<T>> gamma) const {
  const auto& data = abstract_data.get_value<SapLimitConstraintData<T>>();
  *gamma = data.gamma();
}

template <typename T>
void SapLimitConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data = abstract_data.get_value<SapLimitConstraintData<T>>();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const T& ql = parameters_.lower_limit();
  const T& qu = parameters_.upper_limit();
  const VectorX<T>& y = data.y();
  const VectorX<T>& R_inv = data.R_inv();

  G->setZero();
  int i = 0;
  if (ql > -kInf) {
    if (y(i) > 0.0) (*G)(i, i) = R_inv(i);
    ++i;
  }
  if (qu < kInf) {
    if (y(i) > 0.0) (*G)(i, i) = R_inv(i);
  }
}

template <typename T>
void SapLimitConstraint<T>::DoAccumulateGeneralizedImpulses(
    int, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const T& ql = parameters_.lower_limit();
  const T& qu = parameters_.upper_limit();
  // N.B. The NVI guarantees the clique index is correct, and since there is
  // only one, we don't need to use it.
  // For this case the generalized impulses are τ = Jᵀ⋅γ.
  int i = 0;
  // Since this constraint defines the constraint velocity (through its
  // Jacobian) as positive when the state moves away from the limit, then
  // impulses are positive only when the constraint pushes the state away from
  // the limit. This leads to the different signs below for lower and upper
  // limit.
  if (ql > -kInf) (*tau)(clique_dof_) += gamma(i++);  // Lower.
  if (qu < kInf) (*tau)(clique_dof_) -= gamma(i);     // Upper.
}

template <typename T>
std::unique_ptr<SapConstraint<double>> SapLimitConstraint<T>::DoToDouble()
    const {
  const typename SapLimitConstraint<T>::Parameters& p = parameters_;
  SapLimitConstraint<double>::Parameters p_to_double(
      ExtractDoubleOrThrow(p.lower_limit()),
      ExtractDoubleOrThrow(p.upper_limit()),
      ExtractDoubleOrThrow(p.stiffness()),
      ExtractDoubleOrThrow(p.dissipation_time_scale()), p.beta());
  // N.B. Limit constraints always act on a single clique.
  const int clique = this->first_clique();
  const int clique_nv = this->num_velocities(0);
  return std::make_unique<SapLimitConstraint<double>>(
      clique, clique_dof(), clique_nv, ExtractDoubleOrThrow(position()),
      std::move(p_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapLimitConstraint);
