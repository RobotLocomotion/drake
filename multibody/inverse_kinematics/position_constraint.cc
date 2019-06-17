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
    systems::Context<double>* context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions(),
                          p_AQ_lower, p_AQ_upper),
      plant_double_(plant),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      context_double_{context},
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr),
      use_autodiff_{false} {
  if (context == nullptr) throw std::invalid_argument("context is nullptr.");
}

PositionConstraint::PositionConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    systems::Context<AutoDiffXd>* context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions(),
                          p_AQ_lower, p_AQ_upper),
      plant_double_(nullptr),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      context_double_{nullptr},
      plant_autodiff_(plant),
      context_autodiff_(context),
      use_autodiff_{true} {
  if (context == nullptr) throw std::invalid_argument("context is nullptr.");
}

template <typename T>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameA_index, const FrameIndex frameB_index,
                   const Eigen::Vector3d p_BQ,
                   const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) {
  y->resize(3);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameA = plant.get_frame(frameA_index);
  const Frame<T>& frameB = plant.get_frame(frameB_index);
  plant.CalcPointsPositions(*context, frameB, p_BQ.cast<T>(), frameA, y);
}

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  if (use_autodiff_) {
    AutoDiffVecXd y_t;
    Eval(math::initializeAutoDiff(x), &y_t);
    *y = math::autoDiffToValueMatrix(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, frameB_index_,
                  p_BQ_, x, y);
  }
}

void PositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  y->resize(3);
  if (!use_autodiff_) {
    UpdateContextConfiguration(context_double_, *plant_double_,
                               math::autoDiffToValueMatrix(x));
    const Frame<double>& frameA = plant_double_->get_frame(frameA_index_);
    const Frame<double>& frameB = plant_double_->get_frame(frameB_index_);
    Eigen::Vector3d p_AQ{};
    plant_double_->CalcPointsPositions(*context_double_, frameB, p_BQ_, frameA,
                                       &p_AQ);
    Eigen::MatrixXd Jq_V_ABq(6, plant_double_->num_positions());
    plant_double_->CalcJacobianSpatialVelocity(
        *context_double_, JacobianWrtVariable::kQDot, frameB, p_BQ_, frameA,
        frameA, &Jq_V_ABq);
    *y = math::initializeAutoDiffGivenGradientMatrix(
        p_AQ, Jq_V_ABq.bottomRows<3>() * math::autoDiffToGradientMatrix(x));
  } else {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameA_index_,
                  frameB_index_, p_BQ_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
