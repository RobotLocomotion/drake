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
SapLimitConstraint<T>::SapLimitConstraint(int clique, int clique_dof,
                                          int clique_nv, const T& q0,
                                          Parameters parameters)
    : SapConstraint<T>(clique,
                       CalcConstraintFunction(q0, parameters.lower_limit(),
                                              parameters.upper_limit()),
                       CalcConstraintJacobian(clique_dof, clique_nv,
                                              parameters.lower_limit(),
                                              parameters.upper_limit())),
      parameters_(std::move(parameters)) {}

template <typename T>
VectorX<T> SapLimitConstraint<T>::CalcBiasTerm(const T& time_step,
                                               const T&) const {
  return -this->constraint_function() /
         (time_step + parameters_.dissipation_time_scale());
}

template <typename T>
VectorX<T> SapLimitConstraint<T>::CalcDiagonalRegularization(
    const T& time_step, const T& wi) const {
  using std::max;

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor =
      parameters_.beta() * parameters_.beta() / (4.0 * M_PI * M_PI);

  const T& k = parameters_.stiffness();
  const T& taud = parameters_.dissipation_time_scale();

  const T R = max(beta_factor * wi, 1.0 / (time_step * k * (time_step + taud)));

  return VectorX<T>::Constant(this->num_constraint_equations(), R);
}

template <typename T>
void SapLimitConstraint<T>::Project(const Eigen::Ref<const VectorX<T>>& y,
                                    const Eigen::Ref<const VectorX<T>>&,
                                    EigenPtr<VectorX<T>> gamma,
                                    MatrixX<T>* dPdy) const {
  DRAKE_DEMAND(gamma->size() == this->num_constraint_equations());
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const T& ql = parameters_.lower_limit();
  const T& qu = parameters_.upper_limit();

  *gamma = y.array().max(0.0);
  if (dPdy != nullptr) {
    const int nk = this->num_constraint_equations();
    // Resizing is no-op if already the proper size.
    (*dPdy) = MatrixX<T>::Zero(nk, nk);
    int i = 0;
    if (ql > -kInf) {
      if (y(i) > 0.0) (*dPdy)(i, i) = 1;
      i++;
    }
    if (qu < kInf) {
      if (y(i) > 0.0) (*dPdy)(i, i) = 1;
    }
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapLimitConstraint)
