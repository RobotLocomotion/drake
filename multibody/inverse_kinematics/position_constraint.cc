#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
PositionConstraint::PositionConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    systems::Context<double>* plant_context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions(),
                          p_AQ_lower, p_AQ_upper),
      plant_double_(plant),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      context_double_{plant_context},
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr) {
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
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions(),
                          p_AQ_lower, p_AQ_upper),
      plant_double_(nullptr),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      context_double_{nullptr},
      plant_autodiff_(plant),
      context_autodiff_(plant_context) {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
}

void EvalConstraintGradient(const systems::Context<double>& context,
                            const MultibodyPlant<double>& plant,
                            const Frame<double>& frameA,
                            const Frame<double>& frameB,
                            const Eigen::Vector3d& p_AQ,
                            const Eigen::Vector3d& p_BQ,
                            const Eigen::Ref<const AutoDiffVecXd>& x,
                            AutoDiffVecXd* y) {
  Eigen::Matrix3Xd Jq_V_ABq(3, plant.num_positions());
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frameB, p_BQ, frameA, frameA,
                                          &Jq_V_ABq);
  *y = math::InitializeAutoDiff(p_AQ, Jq_V_ABq * math::ExtractGradient(x));
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameA_index, const FrameIndex frameB_index,
                   const Eigen::Vector3d& p_BQ,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(3);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameA = plant.get_frame(frameA_index);
  const Frame<T>& frameB = plant.get_frame(frameB_index);
  Vector3<T> p_AQ;
  plant.CalcPointsPositions(*context, frameB, p_BQ.cast<T>(), frameA, &p_AQ);
  if constexpr (std::is_same_v<T, S>) {
    *y = p_AQ;
  } else {
    EvalConstraintGradient(*context, plant, frameA, frameB, p_AQ, p_BQ, x, y);
  }
}

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(math::InitializeAutoDiff(x), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, frameB_index_,
                  p_BQ_, x, y);
  }
}

void PositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  if (!use_autodiff()) {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, frameB_index_,
                  p_BQ_, x, y);
  } else {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameA_index_,
                  frameB_index_, p_BQ_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
