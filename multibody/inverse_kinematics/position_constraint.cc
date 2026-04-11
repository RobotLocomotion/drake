#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include <utility>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
PositionConstraint::~PositionConstraint() = default;

// Double version. No frameAbar.
PositionConstraint::PositionConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<double>& frameB, std::optional<Eigen::Vector3d> p_BQ,
    systems::Context<double>* plant_context)
    : PositionConstraint(plant, frameA, std::nullopt, p_AQ_lower, p_AQ_upper,
                         frameB, std::move(p_BQ), plant_context) {}

// Double version.
PositionConstraint::PositionConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameAbar,
    const std::optional<math::RigidTransformd>& X_AbarA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<double>& frameB, std::optional<Eigen::Vector3d> p_BQ,
    systems::Context<double>* plant_context)
    : PositionConstraint(plant, frameAbar, X_AbarA, p_AQ_lower, p_AQ_upper,
                         frameB, std::move(p_BQ), plant_context, 0) {}

// Autodiff version.
PositionConstraint::PositionConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameAbar,
    const std::optional<math::RigidTransformd>& X_AbarA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<AutoDiffXd>& frameB, std::optional<Eigen::Vector3d> p_BQ,
    systems::Context<AutoDiffXd>* plant_context)
    : PositionConstraint(plant, frameAbar, X_AbarA, p_AQ_lower, p_AQ_upper,
                         frameB, std::move(p_BQ), plant_context, 0) {}

// AutoDiff version. No frameAbar.
PositionConstraint::PositionConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<AutoDiffXd>& frameB, std::optional<Eigen::Vector3d> p_BQ,
    systems::Context<AutoDiffXd>* plant_context)
    : PositionConstraint(plant, frameA, std::nullopt, p_AQ_lower, p_AQ_upper,
                         frameB, std::move(p_BQ), plant_context, 0) {}

namespace {

void EvalConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant, const Frame<double>& frameAbar,
    const math::RigidTransformd& X_AAbar, const Frame<double>& frameB,
    const Eigen::Vector3d& p_AQ, const Eigen::Vector3d& p_BQ,
    bool p_BQ_is_decision_variable, const Eigen::Ref<const AutoDiffVecXd>& x,
    AutoDiffVecXd* y) {
  Eigen::Matrix3Xd Jq_V_AbarBq(3, plant.num_positions());
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frameB, p_BQ, frameAbar, frameAbar,
                                          &Jq_V_AbarBq);
  if (p_BQ_is_decision_variable) {
    // dy/dq = R_AAbar * Jq_V_AbarBq
    // dy/dp_BQ = R_AAbar * R_AbarB
    Eigen::Matrix3Xd dydx(3, x.size());
    dydx.leftCols(plant.num_positions()) =
        X_AAbar.rotation().matrix() * Jq_V_AbarBq;
    const math::RigidTransformd X_AbarB =
        plant.CalcRelativeTransform(context, frameAbar, frameB);
    dydx.rightCols<3>() =
        X_AAbar.rotation().matrix() * X_AbarB.rotation().matrix();
    const Eigen::Matrix3Xd gradient = dydx * math::ExtractGradient(x);
    *y = math::InitializeAutoDiff(p_AQ, gradient);
  } else {
    *y = math::InitializeAutoDiff(
        p_AQ,
        X_AAbar.rotation().matrix() * Jq_V_AbarBq * math::ExtractGradient(x));
  }
}

// The supported template are
// T = double, S = double
// T = double, S = AutoDiffXd
// T = AutoDiffXd, S = AutoDiffXd
template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameAbar_index,
                   const math::RigidTransformd& X_AAbar,
                   const FrameIndex frameB_index,
                   const std::optional<Eigen::Vector3d>& p_BQ,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(3);
  Vector3<T> p_BQ_T;
  Eigen::Vector3d p_BQ_value;
  const bool p_BQ_is_decision_variable = !p_BQ.has_value();
  if (p_BQ_is_decision_variable) {
    const VectorX<S> q = x.head(plant.num_positions());
    UpdateContextConfiguration(context, plant, q);
    if constexpr (std::is_same_v<S, double>) {
      p_BQ_value = x.template tail<3>();
    } else {
      // x is AutoDiffXd
      p_BQ_value(0) = x(plant.num_positions()).value();
      p_BQ_value(1) = x(plant.num_positions() + 1).value();
      p_BQ_value(2) = x(plant.num_positions() + 2).value();
    }
    if constexpr (std::is_same_v<T, S>) {
      p_BQ_T = x.template tail<3>();
    } else {
      // S != T --> T is double. Assign double to double.
      p_BQ_T = p_BQ_value;
    }
  } else {
    p_BQ_value = p_BQ.value();
    p_BQ_T = p_BQ_value;
    UpdateContextConfiguration(context, plant, x);
  }

  const Frame<T>& frameAbar = plant.get_frame(frameAbar_index);
  const Frame<T>& frameB = plant.get_frame(frameB_index);
  Vector3<T> p_AbarQ;
  plant.CalcPointsPositions(*context, frameB, p_BQ_T, frameAbar, &p_AbarQ);
  if constexpr (std::is_same_v<T, S>) {
    *y = X_AAbar.cast<S>() * p_AbarQ;
  } else {
    EvalConstraintGradient(*context, plant, frameAbar, X_AAbar, frameB,
                           X_AAbar * p_AbarQ, p_BQ_value,
                           p_BQ_is_decision_variable, x, y);
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
namespace {

template <typename T, typename U>
const MultibodyPlant<T>* MaybeMatchScalar(const MultibodyPlant<U>* instance) {
  if constexpr (std::is_same_v<T, U>) {
    return instance;
  } else {
    return nullptr;
  }
}

template <typename T, typename U>
systems::Context<T>* MaybeMatchScalar(systems::Context<U>* instance) {
  if constexpr (std::is_same_v<T, U>) {
    return instance;
  } else {
    return nullptr;
  }
}

}  // namespace

template <typename T>
PositionConstraint::PositionConstraint(
    const MultibodyPlant<T>* plant, const Frame<T>& frameAbar,
    const std::optional<math::RigidTransformd>& X_AbarA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper, const Frame<T>& frameB,
    const std::optional<Eigen::Vector3d>& p_BQ,
    systems::Context<T>* plant_context, int)
    : solvers::Constraint(
          3,
          RefFromPtrOrThrow(plant).num_positions() + (p_BQ.has_value() ? 0 : 3),
          p_AQ_lower, p_AQ_upper),
      plant_double_(MaybeMatchScalar<double>(plant)),
      frameAbar_index_(frameAbar.index()),
      X_AAbar_{X_AbarA.has_value() ? X_AbarA.value().inverse()
                                   : math::RigidTransformd::Identity()},
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      context_double_{MaybeMatchScalar<double>(plant_context)},
      plant_autodiff_(MaybeMatchScalar<AutoDiffXd>(plant)),
      context_autodiff_(MaybeMatchScalar<AutoDiffXd>(plant_context)) {
  if (plant == nullptr) {
    throw std::invalid_argument("PositionConstraint(): plant is nullptr.");
  }
  if (plant_context == nullptr) {
    throw std::invalid_argument(
        "PositionConstraint(): plant_context is nullptr.");
  }
}

}  // namespace multibody
}  // namespace drake
