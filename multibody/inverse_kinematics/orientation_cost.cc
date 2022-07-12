#include "drake/multibody/inverse_kinematics/orientation_cost.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {

OrientationCost::OrientationCost(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameAbar,
    const math::RotationMatrix<double>& R_AbarA, const Frame<double>& frameBbar,
    const math::RotationMatrix<double>& R_BbarB,
    double c, systems::Context<double>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      plant_double_(plant),
      frameAbar_index_(frameAbar.index()),
      R_AAbar_{R_AbarA.inverse()},
      frameBbar_index_(frameBbar.index()),
      R_BbarB_{R_BbarB},
      c_{c},
      context_double_{plant_context},
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr) {
  if (plant_context == nullptr)
    throw std::invalid_argument(
        "OrientationCost(): plant_context is nullptr.");
}

OrientationCost::OrientationCost(const MultibodyPlant<AutoDiffXd>* const plant,
                                 const Frame<AutoDiffXd>& frameAbar,
                                 const math::RotationMatrix<double>& R_AbarA,
                                 const Frame<AutoDiffXd>& frameBbar,
                                 const math::RotationMatrix<double>& R_BbarB,
                                 double c,
                                 systems::Context<AutoDiffXd>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      plant_double_(nullptr),
      frameAbar_index_(frameAbar.index()),
      R_AAbar_{R_AbarA.inverse()},
      frameBbar_index_(frameBbar.index()),
      R_BbarB_{R_BbarB},
      c_{c},
      context_double_{nullptr},
      plant_autodiff_(plant),
      context_autodiff_(plant_context) {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameAbar_index,
                   const math::RotationMatrix<double>& R_AAbar,
                   const FrameIndex frameBbar_index,
                   const math::RotationMatrix<double>& R_BbarB, double c,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(1);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameAbar = plant.get_frame(frameAbar_index);
  const Frame<T>& frameBbar = plant.get_frame(frameBbar_index);
  const math::RotationMatrix<T> R_AbarBbar =
      plant.CalcRelativeRotationMatrix(*context, frameAbar, frameBbar);

  const math::RotationMatrix<T> R_AB = R_AAbar.cast<T>() * R_AbarBbar
                                     * R_BbarB.cast<T>();
  // We have tr(R_AB) = 1 - 2cosθ from the rotation matrix to axis-angle
  // formulas, e.g.:
  // http://motion.pratt.duke.edu/RoboticSystems/3DRotations.html
  // So 1 - cosθ = 1 + (tr(R_AB) - 1)/2
  const T err = 1.0 + (R_AB.matrix().trace() - 1.0)/2.0;
  if constexpr (std::is_same_v<T, S>) {
    (*y)[0] = err;
  } else {
    static_assert(std::is_same_v<T, double>);
    static_assert(std::is_same_v<S, AutoDiffXd>);
    // We have ∂tr(R_AB)/∂q = r_ABᵀ Jq_w_AB, where
    // r = (R₂₃ - R₃₂, R₃₁ - R₁₃, R₁₂ - R₂₁).
    // See orientation_constraint.cc for a full derivation.
    const Eigen::Matrix3d& m = R_AB.matrix();
    const Eigen::Vector3d r_AB{m(1, 2) - m(2, 1), m(2, 0) - m(0, 2),
                          m(0, 1) - m(1, 0)};
  Eigen::MatrixXd Jq_w_AbarBbar(3, plant.num_positions());
  plant.CalcJacobianAngularVelocity(*context, JacobianWrtVariable::kQDot,
                                    frameBbar, frameAbar, frameAbar,
                                    &Jq_w_AbarBbar);
  // Jq_w_AB = Jq_w_AbarBbar_A.
  const Eigen::MatrixXd Jq_w_AB = R_AAbar.matrix() * Jq_w_AbarBbar;
  (*y)[0].value() = err;
  (*y)[0].derivatives() =
      0.5 * r_AB.transpose() * Jq_w_AB * math::ExtractGradient(x);
  }
}

void OrientationCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameAbar_index_, R_AAbar_,
                  frameBbar_index_, R_BbarB_, c_, x, y);
  }
}

void OrientationCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  if (!use_autodiff()) {
    DoEvalGeneric(*plant_double_, context_double_, frameAbar_index_, R_AAbar_,
                  frameBbar_index_, R_BbarB_, c_, x, y);
  } else {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameAbar_index_,
                  R_AAbar_, frameBbar_index_, R_BbarB_, c_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
