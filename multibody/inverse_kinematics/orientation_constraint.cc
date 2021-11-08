#include "drake/multibody/inverse_kinematics/orientation_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
OrientationConstraint::OrientationConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameAbar,
    const math::RotationMatrix<double>& R_AbarA, const Frame<double>& frameBbar,
    const math::RotationMatrix<double>& R_BbarB, double theta_bound,
    systems::Context<double>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(2 * std::cos(theta_bound) + 1), Vector1d(3)),
      plant_double_{plant},
      frameAbar_index_{frameAbar.index()},
      frameBbar_index_{frameBbar.index()},
      R_AbarA_{R_AbarA},
      R_BbarB_{R_BbarB},
      context_double_(plant_context),
      plant_autodiff_{nullptr},
      context_autodiff_{nullptr} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (theta_bound < 0) {
    throw std::invalid_argument(
        "OrientationConstraint: theta_bound should be non-negative.\n");
  }
}

OrientationConstraint::OrientationConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameAbar,
    const math::RotationMatrix<double>& R_AbarA,
    const Frame<AutoDiffXd>& frameBbar,
    const math::RotationMatrix<double>& R_BbarB, double theta_bound,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(2 * std::cos(theta_bound) + 1), Vector1d(3)),
      plant_double_{nullptr},
      frameAbar_index_{frameAbar.index()},
      frameBbar_index_{frameBbar.index()},
      R_AbarA_{R_AbarA},
      R_BbarB_{R_BbarB},
      context_double_(nullptr),
      plant_autodiff_{plant},
      context_autodiff_{plant_context} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (theta_bound < 0) {
    throw std::invalid_argument(
        "OrientationConstraint: theta_bound should be non-negative.\n");
  }
}

void EvalConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant, const Frame<double>& frameAbar,
    const Frame<double>& frameBbar, const math::RotationMatrix<double>& R_AbarA,
    const math::RotationMatrix<double>& R_AB, const Vector3<double>& r_AB,
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
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
  Eigen::MatrixXd Jq_V_AbarBbar(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(
      context, JacobianWrtVariable::kQDot, frameBbar,
      Eigen::Vector3d::Zero() /* p_BQ */, frameAbar, frameAbar, &Jq_V_AbarBbar);
  // Since we're only concerned with the rotational portion,
  // Jq_w_AB = Jq_w_AbarBbar_A.
  const Eigen::MatrixXd Jq_w_AB =
      R_AbarA.inverse().matrix() * Jq_V_AbarBbar.topRows<3>();
  *y = math::InitializeAutoDiff(
      Vector1d(R_AB.matrix().trace()),
      r_AB.transpose() * Jq_w_AB * math::ExtractGradient(x));
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   FrameIndex frameAbar_index, FrameIndex frameBbar_index,
                   const math::RotationMatrix<double>& R_AbarA,
                   const math::RotationMatrix<double>& R_BbarB,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(1);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameAbar = plant.get_frame(frameAbar_index);
  const Frame<T>& frameBbar = plant.get_frame(frameBbar_index);

  const math::RotationMatrix<T> R_AbarBbar =
      plant.CalcRelativeRotationMatrix(*context, frameAbar, frameBbar);

  // Note: The expression below has quantities with different scalar types.
  // The casts from `double` to `T` preserves derivative or symbolic information
  // in R_AbarBbar (if it exists).
  const math::RotationMatrix<T> R_AB = R_AbarA.cast<T>().inverse() * R_AbarBbar
                                     * R_BbarB.cast<T>();
  const Matrix3<T>& m = R_AB.matrix();
  const Vector3<T> r_AB{m(1, 2) - m(2, 1),
                        m(2, 0) - m(0, 2),
                        m(0, 1) - m(1, 0)};
  if constexpr (std::is_same_v<T, S>) {
    (*y)(0) = R_AB.matrix().trace();
  } else {
    EvalConstraintGradient(*context, plant, frameAbar, frameBbar, R_AbarA, R_AB,
                           r_AB, x, y);
  }
}

void OrientationConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(math::InitializeAutoDiff(x), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameAbar_index_,
                  frameBbar_index_, R_AbarA_, R_BbarB_, x, y);
  }
}

void OrientationConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameAbar_index_,
                  frameBbar_index_, R_AbarA_, R_BbarB_, x, y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameAbar_index_,
                  frameBbar_index_, R_AbarA_, R_BbarB_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
