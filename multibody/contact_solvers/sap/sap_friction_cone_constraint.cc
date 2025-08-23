#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"

#include <algorithm>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_eigen.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapFrictionConeConstraint<T>::SapFrictionConeConstraint(
    ContactConfiguration<T> configuration, SapConstraintJacobian<T> J,
    Parameters parameters)
    : SapConstraint<T>(std::move(J),
                       {configuration.objectA, configuration.objectB}),
      parameters_(std::move(parameters)),
      configuration_(std::move(configuration)) {
  DRAKE_DEMAND(parameters_.mu >= 0.0);
  DRAKE_DEMAND(parameters_.stiffness > 0.0);
  DRAKE_DEMAND(parameters_.dissipation_time_scale >= 0.0);
  DRAKE_DEMAND(parameters_.beta >= 0.0);
  DRAKE_DEMAND(parameters_.sigma > 0.0);
  DRAKE_DEMAND(this->jacobian().rows() == 3);
}

template <typename T>
SapFrictionConeConstraintData<T>::SapFrictionConeConstraintData(
    const T& mu, const T& Rt, const T& Rn, const T& vn_hat) {
  using std::sqrt;
  parameters_.mu = mu;
  parameters_.mu_tilde = mu * sqrt(Rt / Rn);
  parameters_.mu_hat = mu * Rt / Rn;
  parameters_.R = Vector3<T>(Rt, Rt, Rn);
  parameters_.R_inv = parameters_.R.cwiseInverse();
  parameters_.Rsqrt = parameters_.R.cwiseSqrt();
  parameters_.Rsqrt_inv = parameters_.Rsqrt.cwiseInverse();
  parameters_.v_hat = Vector3<T>(0, 0, vn_hat);
}

template <typename T>
std::unique_ptr<AbstractValue> SapFrictionConeConstraint<T>::DoMakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  using std::max;
  using std::sqrt;

  // Estimate a w_rms guaranteed to be larger than zero.
  const T w_rms = delassus_estimation.norm() / sqrt(3.0);

  // Compute tangent and normal scaling from Delassus estimation, bounded with
  // w_rms to avoid zero values.
  const T wt =
      max(0.5 * (delassus_estimation(0) + delassus_estimation(1)), w_rms);
  const T wn = max(delassus_estimation(2), w_rms);

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor =
      parameters_.beta * parameters_.beta / (4.0 * M_PI * M_PI);

  const T& k = parameters_.stiffness;
  const T& taud = parameters_.dissipation_time_scale;

  // Regularization parameters.
  const T Rn =
      max(beta_factor * wn, 1.0 / (time_step * k * (time_step + taud)));
  const T Rt = parameters_.sigma * wt;

  // Bias term.
  const T vn_hat = -configuration_.phi / (time_step + taud);

  // Create our return value in a way that avoids extra copies.
  return std::unique_ptr<AbstractValue>(
      new Value<SapFrictionConeConstraintData<T>>(parameters_.mu, Rt, Rn,
                                                  vn_hat));
}

template <typename T>
ContactMode SapFrictionConeConstraint<T>::CalcContactMode(const T& mu,
                                                          const T& mu_hat,
                                                          const T& yr,
                                                          const T& yn) {
  if (yr <= mu * yn) return ContactMode::kStiction;
  if (-mu_hat * yr < yn && yn < yr / mu) return ContactMode::kSliding;
  return ContactMode::kNoContact;
}

template <typename T>
void SapFrictionConeConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>& vc,
    AbstractValue* abstract_data) const {
  auto& data =
      abstract_data->get_mutable_value<SapFrictionConeConstraintData<T>>();

  data.mutable_vc() = vc;
  data.mutable_y() = data.R_inv().asDiagonal() * (data.v_hat() - vc);
  const auto yt = data.y().template head<2>();
  data.mutable_yr() = SoftNorm(yt);
  data.mutable_yn() = data.y()(2);
  data.mutable_t_hat() = yt / data.yr();

  data.mutable_mode() =
      CalcContactMode(data.mu(), data.mu_hat(), data.yr(), data.yn());

  ProjectImpulse(data, &data.mutable_gamma());
}

template <typename T>
void SapFrictionConeConstraint<T>::ProjectImpulse(
    const SapFrictionConeConstraintData<T>& data, Vector3<T>* gamma) const {
  // Analytical projection of y onto the friction cone ℱ using the R norm.
  switch (data.mode()) {
    case ContactMode::kStiction: {
      *gamma = data.y();
      break;
    }
    case ContactMode::kSliding: {
      const T& yr = data.yr();
      const T& yn = data.yn();
      const Vector2<T>& that = data.t_hat();

      const T mu_hat = data.mu_hat();
      const T mu_tilde2 = data.mu_tilde() * data.mu_tilde();
      const T factor = 1.0 / (1.0 + mu_tilde2);

      // Projection P(y).
      const T gn = (yn + mu_hat * yr) * factor;
      const Vector2<T> gt = data.mu() * gn * that;
      *gamma << gt, gn;
      break;
    }
    case ContactMode::kNoContact: {
      gamma->setZero();
      break;
    }
  }
}

template <typename T>
T SapFrictionConeConstraint<T>::DoCalcCost(
    const AbstractValue& abstract_data) const {
  const auto& data =
      abstract_data.get_value<SapFrictionConeConstraintData<T>>();
  const Vector3<T>& R = data.R();
  const Vector3<T>& gamma = data.gamma();
  return 0.5 * gamma.dot(R.asDiagonal() * gamma);
}

template <typename T>
void SapFrictionConeConstraint<T>::DoCalcImpulse(
    const AbstractValue& abstract_data, EigenPtr<VectorX<T>> gamma) const {
  const auto& data =
      abstract_data.get_value<SapFrictionConeConstraintData<T>>();
  *gamma = data.gamma();
}

template <typename T>
void SapFrictionConeConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data =
      abstract_data.get_value<SapFrictionConeConstraintData<T>>();

  // Analytical projection of y onto the friction cone ℱ using the R norm.
  switch (data.mode()) {
    case ContactMode::kStiction: {
      *G = data.R_inv().asDiagonal();
      break;
    }
    case ContactMode::kSliding: {
      const T& mu = data.mu();
      const T& mu_hat = data.mu_hat();
      const T& yr = data.yr();
      const Vector2<T>& that = data.t_hat();

      // We first compute G = dP/dy, with P the SAP projection s.t. γ = P(y).
      const Matrix2<T> P = that * that.transpose();
      const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;
      const T mu_tilde2 = data.mu_tilde() * data.mu_tilde();
      const T factor = 1.0 / (1.0 + mu_tilde2);

      // We split dPdy into separate blocks:
      //
      // dPdy = |dgt_dyt dgt_dyn|
      //        |dgn_dyt dgn_dyn|
      // where dgt_dyt ∈ ℝ²ˣ², dgt_dyn ∈ ℝ², dgn_dyt ∈ ℝ²ˣ¹ and dgn_dyn ∈ ℝ.
      const T& gn = data.gamma()[2];  // Normal impulse.
      const Matrix2<T> dgt_dyt = mu * (gn / yr * Pperp + mu_hat * factor * P);
      const Vector2<T> dgt_dyn = mu * factor * that;
      const RowVector2<T> dgn_dyt = mu_hat * factor * that.transpose();
      const T dgn_dyn = factor;

      G->template topLeftCorner<2, 2>() = dgt_dyt;
      G->template topRightCorner<2, 1>() = dgt_dyn;
      G->template bottomLeftCorner<1, 2>() = dgn_dyt;
      (*G)(2, 2) = dgn_dyn;

      // Multiply by R⁻¹ from the right.
      (*G) *= data.R_inv().asDiagonal();
      break;
    }
    case ContactMode::kNoContact: {
      G->setZero();
      break;
    }
  }
}

template <typename T>
void SapFrictionConeConstraint<T>::DoAccumulateSpatialImpulses(
    int i, const Eigen::Ref<const VectorX<T>>& gamma,
    SpatialForce<T>* F) const {
  const math::RotationMatrix<T>& R_WC = configuration_.R_WC;
  const Vector3<T> f_Bc_W = R_WC * gamma;
  const SpatialForce<T> F_Bc_W(Vector3<T>::Zero(), f_Bc_W);

  if (i == 0) {
    // Object A.
    const Vector3<T> p_CAp_W = -configuration_.p_ApC_W;
    // N.B. F_Ap = F_Ac.Shift(p_CAp) = -F_Bc.Shift(p_CAp)
    *F -= F_Bc_W.Shift(p_CAp_W);
  } else {
    // Object B.
    const Vector3<T> p_CBq_W = -configuration_.p_BqC_W;
    // N.B. F_Bq = F_Bc.Shift(p_CBq)
    *F += F_Bc_W.Shift(p_CBq_W);
    return;
  }
}

template <typename T>
std::unique_ptr<SapConstraint<double>>
SapFrictionConeConstraint<T>::DoToDouble() const {
  const typename SapFrictionConeConstraint<T>::Parameters& p = parameters_;
  SapFrictionConeConstraint<double>::Parameters p_to_double{
      ExtractDoubleOrThrow(p.mu), ExtractDoubleOrThrow(p.stiffness),
      ExtractDoubleOrThrow(p.dissipation_time_scale), p.beta, p.sigma};
  return std::make_unique<SapFrictionConeConstraint<double>>(
      configuration().ToDouble(), this->jacobian().ToDouble(),
      std::move(p_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapFrictionConeConstraint);
