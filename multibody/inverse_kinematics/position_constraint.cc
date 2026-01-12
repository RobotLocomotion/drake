#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {

PositionConstraint::~PositionConstraint() = default;

PositionConstraint::PositionConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    systems::Context<double>* plant_context)
    : PositionConstraint(plant, frameA, std::nullopt, p_AQ_lower, p_AQ_upper,
                         frameB, p_BQ, plant_context) {}

PositionConstraint::PositionConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameAbar,
    const std::optional<math::RigidTransformd>& X_AbarA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    systems::Context<double>* plant_context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions(),
                          p_AQ_lower, p_AQ_upper),
      plant_double_(plant),
      frameAbar_index_(frameAbar.index()),
      X_AAbar_{X_AbarA.has_value() ? X_AbarA.value().inverse()
                                   : math::RigidTransformd::Identity()},
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      context_double_{plant_context},
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr) {
  if (plant == nullptr) {
    throw std::invalid_argument("PositionConstraint(): plant is nullptr.");
  }
  if (plant_context == nullptr) {
    throw std::invalid_argument(
        "PositionConstraint(): plant_context is nullptr.");
  }
}

PositionConstraint::PositionConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameAbar,
    const std::optional<math::RigidTransformd>& X_AbarA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions(),
                          p_AQ_lower, p_AQ_upper),
      plant_double_(nullptr),
      frameAbar_index_(frameAbar.index()),
      X_AAbar_{X_AbarA.has_value() ? X_AbarA.value().inverse()
                                   : math::RigidTransformd::Identity()},
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      context_double_{nullptr},
      plant_autodiff_(plant),
      context_autodiff_(plant_context) {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
}

PositionConstraint::PositionConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    systems::Context<AutoDiffXd>* plant_context)
    : PositionConstraint(plant, frameA, std::nullopt, p_AQ_lower, p_AQ_upper,
                         frameB, p_BQ, plant_context) {}

namespace {

void EvalConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant, const Frame<double>& frameAbar,
    const math::RigidTransformd& X_AAbar, const Frame<double>& frameB,
    const Eigen::Vector3d& p_AQ, const Eigen::Vector3d& p_BQ,
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  Eigen::Matrix3Xd Jq_V_AbarBq(3, plant.num_positions());
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frameB, p_BQ, frameAbar, frameAbar,
                                          &Jq_V_AbarBq);
  *y =
      math::InitializeAutoDiff(p_AQ, X_AAbar.rotation().matrix() * Jq_V_AbarBq *
                                         math::ExtractGradient(x));
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameAbar_index,
                   const math::RigidTransformd& X_AAbar,
                   const FrameIndex frameB_index, const Eigen::Vector3d& p_BQ,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(3);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameAbar = plant.get_frame(frameAbar_index);
  const Frame<T>& frameB = plant.get_frame(frameB_index);
  Vector3<T> p_AbarQ;
  plant.CalcPointsPositions(*context, frameB, p_BQ.cast<T>(), frameAbar,
                            &p_AbarQ);
  if constexpr (std::is_same_v<T, S>) {
    *y = X_AAbar.cast<S>() * p_AbarQ;
  } else {
    EvalConstraintGradient(*context, plant, frameAbar, X_AAbar, frameB,
                           X_AAbar * p_AbarQ, p_BQ, x, y);
  }
}

}  // namespace

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameAbar_index_, X_AAbar_,
                  frameB_index_, p_BQ_, x, y);
  }
}

void PositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  if (!use_autodiff()) {
    DoEvalGeneric(*plant_double_, context_double_, frameAbar_index_, X_AAbar_,
                  frameB_index_, p_BQ_, x, y);
  } else {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameAbar_index_,
                  X_AAbar_, frameB_index_, p_BQ_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
