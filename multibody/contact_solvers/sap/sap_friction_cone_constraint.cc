#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"

#include <algorithm>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapFrictionConeConstraint<T>::SapFrictionConeConstraint(int clique,
                                                        MatrixBlock<T> J,
                                                        const T& phi0,
                                                        const Parameters& p)
    : SapConstraint<T>(clique, Vector3<T>(0.0, 0.0, phi0), std::move(J)),
      parameters_(p),
      phi0_(phi0) {
  DRAKE_DEMAND(clique >= 0);
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(p.stiffness > 0.0);
  DRAKE_DEMAND(p.dissipation_time_scale >= 0.0);
  DRAKE_DEMAND(p.beta >= 0.0);
  DRAKE_DEMAND(p.sigma > 0.0);
  DRAKE_DEMAND(this->first_clique_jacobian().rows() == 3);
}

template <typename T>
SapFrictionConeConstraint<T>::SapFrictionConeConstraint(
    int clique0, int clique1, MatrixBlock<T> J0, MatrixBlock<T> J1,
    const T& phi0, const Parameters& p)
    : SapConstraint<T>(clique0, clique1, Vector3<T>(0.0, 0.0, phi0),
                       std::move(J0), std::move(J1)),
      parameters_(p),
      phi0_(phi0) {
  DRAKE_DEMAND(clique0 >= 0);
  DRAKE_DEMAND(clique1 >= 0);
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(p.stiffness > 0.0);
  DRAKE_DEMAND(p.dissipation_time_scale >= 0.0);
  DRAKE_DEMAND(p.beta >= 0.0);
  DRAKE_DEMAND(p.sigma > 0.0);
  DRAKE_DEMAND(this->first_clique_jacobian().rows() == 3);
  DRAKE_DEMAND(this->second_clique_jacobian().rows() == 3);
}

template <typename T>
VectorX<T> SapFrictionConeConstraint<T>::CalcBiasTerm(const T& time_step,
                                                      const T&) const {
  const T& taud = parameters_.dissipation_time_scale;
  const T vn_hat = -phi0_ / (time_step + taud);
  return Vector3<T>(0, 0, vn_hat);
}

template <typename T>
VectorX<T> SapFrictionConeConstraint<T>::CalcDiagonalRegularization(
    const T& time_step, const T& wi) const {
  using std::max;

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor =
      parameters_.beta * parameters_.beta / (4.0 * M_PI * M_PI);

  const T& k = parameters_.stiffness;
  const T& taud = parameters_.dissipation_time_scale;

  const T Rn =
      max(beta_factor * wi, 1.0 / (time_step * k * (time_step + taud)));
  const T Rt = parameters_.sigma * wi;
  return Vector3<T>(Rt, Rt, Rn);
}

template <typename T>
void SapFrictionConeConstraint<T>::Project(
    const Eigen::Ref<const VectorX<T>>& y,
    const Eigen::Ref<const VectorX<T>>& R, EigenPtr<VectorX<T>> gamma,
    MatrixX<T>* dPdy) const {
  // Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
  // soft_tolerance. Using the soft norm we define the tangent vector as t̂ =
  // γₜ/‖γₜ‖ₛ, which is well defined event for γₜ = 0. Also gradients are well
  // defined and follow the same equations presented in [Castro et al., 2021]
  // where regular norms are simply replaced by soft norms.
  auto soft_norm = [](const Eigen::Ref<const VectorX<T>>& x) -> T {
    using std::sqrt;
    // TODO(amcastro-tri): consider exposing this as a parameter.
    constexpr double soft_tolerance = 1.0e-7;
    constexpr double soft_tolerance_squared = soft_tolerance * soft_tolerance;
    return sqrt(x.squaredNorm() + soft_tolerance_squared);
  };

  // We assume a regularization of the form R = (Rt, Rt, Rn).
  const T& Rt = R(0);
  const T& Rn = R(2);
  const T mu_hat = mu() * Rt / Rn;

  const auto yt = y.template head<2>();
  const T yr = soft_norm(yt);
  const T yn = y(2);
  const Vector2<T> that = yt / yr;

  if (dPdy != nullptr) {
    dPdy->resize(3, 3);  // no-op if already the proper size.
  }

  // Analytical projection of y onto the friction cone ℱ using the R norm.
  if (yr < mu() * yn) {
    // Region I, stiction.
    *gamma = y;
    if (dPdy) dPdy->setIdentity();
  } else if (-mu_hat * yr < yn && yn <= yr / mu()) {
    // Region II, sliding.

    // Common terms in both the projection and its gradient.
    const T mu_tilde2 = mu() * mu_hat;  // mu_tilde = mu * sqrt(Rt/Rn).
    const T factor = 1.0 / (1.0 + mu_tilde2);

    // Projection P(y).
    const T gn = (yn + mu_hat * yr) * factor;
    const Vector2<T> gt = mu() * gn * that;
    *gamma << gt, gn;

    // Gradient.
    if (dPdy) {
      const Matrix2<T> P = that * that.transpose();
      const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

      // We split dPdy into separate blocks:
      //
      // dPdy = |dgt_dyt dgt_dyn|
      //        |dgn_dyt dgn_dyn|
      // where dgt_dyt ∈ ℝ²ˣ², dgt_dyn ∈ ℝ², dgn_dyt ∈ ℝ²ˣ¹ and dgn_dyn ∈ ℝ.
      const Matrix2<T> dgt_dyt = mu() * (gn / yr * Pperp + mu_hat * factor * P);
      const Vector2<T> dgt_dyn = mu() * factor * that;
      const RowVector2<T> dgn_dyt = mu_hat * factor * that.transpose();
      const T dgn_dyn = factor;

      dPdy->template topLeftCorner<2, 2>() = dgt_dyt;
      dPdy->template topRightCorner<2, 1>() = dgt_dyn;
      dPdy->template bottomLeftCorner<1, 2>() = dgn_dyt;
      (*dPdy)(2, 2) = dgn_dyn;
    }
  } else {  // yn <= -mu_hat * yr
    // Region III, no contact.
    gamma->setZero();
    if (dPdy) dPdy->setZero();
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapFrictionConeConstraint)
