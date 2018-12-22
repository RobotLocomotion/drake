#include "drake/multibody/inverse_kinematics/orientation_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
OrientationConstraint::OrientationConstraint(
    const MultibodyPlant<double>* const plant,
    const Frame<double>& frameAbar, const math::RotationMatrix<double>& R_AbarA,
    const Frame<double>& frameBbar, const math::RotationMatrix<double>& R_BbarB,
    double theta_bound, systems::Context<double>* context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(2 * std::cos(theta_bound) + 1), Vector1d(3)),
      plant_{RefFromPtrOrThrow(plant)},
      frameAbar_index_{frameAbar.index()},
      frameBbar_index_{frameBbar.index()},
      R_AbarA_{R_AbarA},
      R_BbarB_{R_BbarB},
      context_(context) {
  if (context == nullptr) throw std::invalid_argument("context is nullptr.");
  if (theta_bound < 0) {
    throw std::invalid_argument(
        "OrientationConstraint: theta_bound should be non-negative.\n");
  }
}

void OrientationConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  // TODO(avalenzu): Re-work to avoid round-trip through AutoDiffXd (#10205).
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void OrientationConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  // The constraint function is
  //  g(q) = tr(R_AB(q)).
  // To derive the Jacobian of g, ∂g/∂q, we first differentiate
  // w.r.t. time to obtain
  //   ġ = tr(Ṙ_AB),
  // where we have dropped the dependence on q for readability. Next we expand
  // Ṙ_AB to yield
  //   ġ = tr(ω̂_AB R_AB).
  //         ⎛⎡ 0  -ω₃  ω₂⎤⎡R₁₁ R₁₂ R₁₃⎤⎞
  //     = tr⎜⎢ ω₃  0  -ω₁⎥⎢R₂₁ R₂₂ R₂₃⎥⎟
  //         ⎝⎣-ω₂  ω₁  0 ⎦⎣R₃₁ R₃₂ R₃₃⎦⎠
  //     = (R₂₃ - R₃₂)ω₁ + (R₃₁ - R₁₃)ω₂ + (R₁₂ - R₂₁)ω₃,
  // where we have dropped the _AB for brevity. Let
  //   r = (R₂₃ - R₃₂, R₃₁ - R₁₃, R₁₂ - R₂₁)ᵀ.
  // Then,
  //   ġ = r_ABᵀ ω_AB.
  // The angular velocity of B relative to A can be written as
  //   ω_AB = Jq_w_AB(q) q̇,
  // where Jq_w_AB is the Jacobian of ω_AB with respect to q̇. Therefore,
  //   ġ = r_ABᵀ Jq_w_AB q̇.
  // By the chain rule, ġ = ∂g/∂q q̇. Comparing this with the previous equation,
  // we see that
  //   ∂g/∂q = r_ABᵀ Jq_w_AB.
  y->resize(1);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  const Frame<double>& frameAbar = plant_.get_frame(frameAbar_index_);
  const Frame<double>& frameBbar = plant_.get_frame(frameBbar_index_);
  //
  const Matrix3<double> R_AbarBbar =
      plant_.CalcRelativeTransform(*context_, frameAbar, frameBbar).linear();
  const Matrix3<double> R_AB =
      R_AbarA_.inverse().matrix() * R_AbarBbar * R_BbarB_.matrix();
  Eigen::MatrixXd Jq_V_AbarBbar(6, plant_.num_positions());
  plant_.CalcJacobianSpatialVelocity(
      *context_, JacobianWrtVariable::kQDot, frameBbar,
      Eigen::Vector3d::Zero() /* p_BQ */, frameAbar, frameAbar, &Jq_V_AbarBbar);
  // Since we're only concerned with the rotational portion,
  // Jq_w_AB = Jq_w_AbarBbar_A.
  const Eigen::MatrixXd Jq_w_AB =
      R_AbarA_.inverse().matrix() * Jq_V_AbarBbar.topRows<3>();
  const Vector3<double> r_AB{R_AB(1, 2) - R_AB(2, 1), R_AB(2, 0) - R_AB(0, 2),
                             R_AB(0, 1) - R_AB(1, 0)};
  *y = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(R_AB.trace()),
      r_AB.transpose() * Jq_w_AB * math::autoDiffToGradientMatrix(x));
}

}  // namespace multibody
}  // namespace drake
