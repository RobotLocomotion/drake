#include "drake/multibody/inverse_kinematics/com_position_constraint.h"

#include <utility>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
ComPositionConstraint::ComPositionConstraint(
    const MultibodyPlant<double>* const plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances,
    const Frame<double>& expressed_frame,
    systems::Context<double>* plant_context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions() + 3,
                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      plant_double_{plant},
      model_instances_{model_instances},
      expressed_frame_index_{expressed_frame.index()},
      context_double_{plant_context},
      plant_autodiff_{nullptr},
      context_autodiff_{nullptr} {
  // TODO(hongkai.dai): allow model_instances to have value when #14916 is
  // resolved.
  if (model_instances_.has_value()) {
    throw std::runtime_error(
        "ComPositionConstraint: currently we only accept std::nullopt as "
        "model_instances");
  }
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  this->set_description(plant->GetSystemName() + " CoM position constraint");
  // TODO(hongkai.dai): set the sparsity pattern. This constraint only depends
  // on the generalized positions in model_instances.
}

ComPositionConstraint::ComPositionConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances,
    const Frame<AutoDiffXd>& expressed_frame,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions() + 3,
                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      plant_double_{nullptr},
      model_instances_{std::move(model_instances)},
      expressed_frame_index_{expressed_frame.index()},
      context_double_{nullptr},
      plant_autodiff_{plant},
      context_autodiff_{plant_context} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  this->set_description(plant->GetSystemName() + " CoM position constraint");
  // TODO(hongkai.dai): set the sparsity pattern. This constraint only depends
  // on the generalized positions in model_instances.
}

// We can explicitly evaluate the gradient of the constraint with
// MultibodyPlant<double>, using the Jacobian function in MBP<double>.
void EvalConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    const Frame<double>& expressed_frame, const Eigen::Vector3d& p_EC,
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  // TODO(hongkai.dai): compute the CoM Jacobian with model_instances when
  // #14916 is resolved.
  unused(model_instances);
  Eigen::Matrix3Xd Jq_V_EC(3, plant.num_positions());
  plant.CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kQDot, expressed_frame, expressed_frame,
      &Jq_V_EC);
  const Eigen::Vector3d y_val = p_EC - math::ExtractValue(x.tail<3>());
  Eigen::Matrix3Xd dy_dx(3, plant.num_positions() + 3);
  dy_dx << Jq_V_EC, -Eigen::Matrix3d::Identity();
  *y = math::InitializeAutoDiff(y_val, dy_dx * math::ExtractGradient(x));
}

// (T, S) can be (double, double), (double, AutoDiffXd) or (AutoDiffXd,
// AutoDiffXd)
template <typename T, typename S>
void DoEvalGeneric(
    const MultibodyPlant<T>& plant, systems::Context<T>* context,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    FrameIndex expressed_frame_index, const Eigen::Ref<const VectorX<S>>& x,
    VectorX<S>* y) {
  y->resize(3);
  UpdateContextConfiguration(context, plant, x.head(plant.num_positions()));

  // Position of the CoM (C) expressed in the world frame (W).
  Vector3<T> p_WC;
  if (model_instances.has_value()) {
    p_WC = plant.CalcCenterOfMassPositionInWorld(*context,
                                                 model_instances.value());
  } else {
    p_WC = plant.CalcCenterOfMassPositionInWorld(*context);
  }
  const math::RigidTransform<T> X_EW = plant.CalcRelativeTransform(
      *context, plant.get_frame(expressed_frame_index), plant.world_frame());
  const Vector3<T> p_EC = X_EW * p_WC;
  if constexpr (std::is_same_v<T, S>) {
    // T=S = double or T=S=AutoDiffXd
    *y = p_EC - x.template tail<3>();
  } else {
    // T = double and S = AutoDiffXd. We compute the gradient using the Jacobian
    // function from MBP<double>.
    EvalConstraintGradient(*context, plant, model_instances,
                           plant.get_frame(expressed_frame_index), p_EC, x, y);
  }
}

void ComPositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, model_instances_,
                  expressed_frame_index_, x, y);
  }
}

void ComPositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, model_instances_,
                  expressed_frame_index_, x, y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, model_instances_,
                  expressed_frame_index_, x, y);
  }
}
}  // namespace multibody
}  // namespace drake
