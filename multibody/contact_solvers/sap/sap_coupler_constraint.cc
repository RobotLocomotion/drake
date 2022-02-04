#include "drake/multibody/contact_solvers/sap/sap_coupler_constraint.h"

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapCouplerConstraint<T>::SapCouplerConstraint(const Parameters& p, int clique,
                                              const MatrixX<T>& J, const T& g0)
    : SapConstraint<T>(clique, J), parameters_(p), g0_(g0) {
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 1);
}

template <typename T>
SapCouplerConstraint<T>::SapCouplerConstraint(const Parameters& p, int clique0,
                                              int clique1, const MatrixX<T>& J0,
                                              const MatrixX<T>& J1, const T& g0)
    : SapConstraint<T>(clique0, clique1, J0, J1), parameters_(p), g0_(g0) {
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 1);
  DRAKE_DEMAND(this->clique1_jacobian().rows() == 1);
}

template <typename T>
VectorX<T> SapCouplerConstraint<T>::CalcBiasTerm(const T& time_step,
                                                 const T&) const {
  const T& taud = parameters_.dissipation_time_scale;
  const T v_hat = -g0_ / (time_step + taud);
  return Vector1<T>(v_hat);
}

template <typename T>
VectorX<T> SapCouplerConstraint<T>::CalcDiagonalRegularization(
    const T& time_step, const T& wi) const {
  using std::max;

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor =
      parameters_.beta * parameters_.beta / (4.0 * M_PI * M_PI);

  const T& k = parameters_.stiffness;
  const T& taud = parameters_.dissipation_time_scale;

  const T R = max(beta_factor * wi, 1.0 / (time_step * k * (time_step + taud)));
  return Vector1<T>(R);
}

template <typename T>
void SapCouplerConstraint<T>::Project(const Eigen::Ref<const VectorX<T>>& y,
                                      const Eigen::Ref<const VectorX<T>>&,
                                      EigenPtr<VectorX<T>> gamma,
                                      MatrixX<T>* dPdy) const {
  // For this constraint the projection operator is the identity operator.
  *gamma = y;
  if (dPdy != nullptr) {
    dPdy->resize(1, 1);  // no-op if already the proper size.
    dPdy->setIdentity();
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapCouplerConstraint)
