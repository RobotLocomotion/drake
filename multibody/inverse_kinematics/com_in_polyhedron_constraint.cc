#include "drake/multibody/inverse_kinematics/com_in_polyhedron_constraint.h"

#include <utility>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
ComInPolyhedronConstraint::ComInPolyhedronConstraint(
    const MultibodyPlant<double>* plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances,
    const Frame<double>& expressed_frame,
    const Eigen::Ref<const Eigen::MatrixX3d>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    systems::Context<double>* plant_context)
    : solvers::Constraint(A.rows(), RefFromPtrOrThrow(plant).num_positions(),
                          lb, ub),
      plant_double_{plant},
      model_instances_{std::move(model_instances)},
      expressed_frame_index_{expressed_frame.index()},
      A_{A},
      context_double_{plant_context},
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr) {
  if (plant_context == nullptr)
    throw std::invalid_argument(
        "ComInPolyhedronConstraint: plant_context is nullptr.");
  this->set_description("com in polyhedron constraint");
  // TODO(hongkai.dai): remove this error when #14916 is resolved.
  if (model_instances_.has_value()) {
    throw std::invalid_argument(
        "ComInPolyhedronConstraint: model_instances has to be std::nullopt "
        "until issue 14916 is resolved.");
  }
}

ComInPolyhedronConstraint::ComInPolyhedronConstraint(
    const MultibodyPlant<AutoDiffXd>* plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances,
    const Frame<AutoDiffXd>& expressed_frame,
    const Eigen::Ref<const Eigen::MatrixX3d>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(A.rows(), RefFromPtrOrThrow(plant).num_positions(),
                          lb, ub),
      plant_double_(nullptr),
      model_instances_{std::move(model_instances)},
      expressed_frame_index_{expressed_frame.index()},
      A_{A},
      context_double_{nullptr},
      plant_autodiff_{plant},
      context_autodiff_{plant_context} {
  if (plant_context == nullptr)
    throw std::invalid_argument(
        "ComInPolyhedronConstraint: plant_context is nullptr.");
  this->set_description("com in polyhedron constraint");
  if (model_instances_.has_value() && model_instances_.value().empty()) {
    throw std::invalid_argument(
        "ComInPolyhedronConstraint: model_instances is an empty vector.");
  }
}

void EvalConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    const Frame<double>& expressed_frame, const Eigen::Vector3d& p_EC,
    const Eigen::MatrixX3d& A, const Eigen::Ref<const AutoDiffVecXd>& x,
    AutoDiffVecXd* y) {
  // TODO(hongkai.dai): compute the CoM Jacobian with model_instances when
  // #14916 is resolved.
  unused(model_instances);
  Eigen::Matrix3Xd Jq_V_EC(3, plant.num_positions());
  plant.CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kQDot, expressed_frame, expressed_frame,
      &Jq_V_EC);
  const Eigen::VectorXd y_val = A * p_EC;
  Eigen::MatrixXd dy_dx(A.rows(), plant.num_positions());
  dy_dx << A * Jq_V_EC;
  *y = math::InitializeAutoDiff(y_val, dy_dx * math::ExtractGradient(x));
}

// (T, S) can be (double, double), (double, AutoDiffXd) and (AutoDiffXd,
// AutoDiffXd)
template <typename T, typename S>
void DoEvalGeneric(
    const MultibodyPlant<T>& plant, systems::Context<T>* context,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    const FrameIndex expressed_frame_index, const Eigen::MatrixX3d& A,
    const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(A.rows());
  UpdateContextConfiguration(context, plant, x);

  // Position of CoM (C) expressed in the world frame W.
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
    // T=S=double or T=S=AutoDiffXd;
    *y = A * p_EC;
  } else {
    // T = double and S = AutoDiffXd. We compute the gradient using the Jacobian
    // function from MBP<double>.
    EvalConstraintGradient(*context, plant, model_instances,
                           plant.get_frame(expressed_frame_index), p_EC, A, x,
                           y);
  }
}

void ComInPolyhedronConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, model_instances_,
                  expressed_frame_index_, A_, x, y);
  }
}

void ComInPolyhedronConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, model_instances_,
                  expressed_frame_index_, A_, x, y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, model_instances_,
                  expressed_frame_index_, A_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
