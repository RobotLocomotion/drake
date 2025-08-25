#include "drake/multibody/inverse_kinematics/position_cost.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::PtrOrThrow;
using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {

PositionCost::PositionCost(const MultibodyPlant<double>* const plant,
                           const Frame<double>& frameA,
                           const Eigen::Ref<const Eigen::Vector3d>& p_AP,
                           const Frame<double>& frameB,
                           const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                           const Eigen::Ref<const Eigen::Matrix3d>& C,
                           systems::Context<double>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      constraint_(plant, frameA, p_AP, p_AP, frameB, p_BQ,
                  PtrOrThrow(plant_context,
                             "PositionCost(): plant_context is nullptr")),
      C_{C} {}

PositionCost::PositionCost(const MultibodyPlant<AutoDiffXd>* const plant,
                           const Frame<AutoDiffXd>& frameA,
                           const Eigen::Ref<const Eigen::Vector3d>& p_AP,
                           const Frame<AutoDiffXd>& frameB,
                           const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                           const Eigen::Ref<const Eigen::Matrix3d>& C,
                           systems::Context<AutoDiffXd>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      constraint_(plant, frameA, p_AP, p_AP, frameB, p_BQ,
                  PtrOrThrow(plant_context,
                             "PositionCost(): plant_context is nullptr")),
      C_{C} {}

PositionCost::~PositionCost() = default;

void PositionCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                          Eigen::VectorXd* y) const {
  y->resize(1);
  Eigen::VectorXd p_AQ;
  constraint_.Eval(x, &p_AQ);
  const Eigen::VectorXd& p_AP = constraint_.lower_bound();
  const Eigen::VectorXd err = p_AQ - p_AP;
  (*y)[0] = err.dot(C_ * err);
}

void PositionCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                          AutoDiffVecXd* y) const {
  y->resize(1);
  AutoDiffVecXd p_AQ;
  constraint_.Eval(x, &p_AQ);
  const Eigen::VectorXd& p_AP = constraint_.lower_bound();
  const AutoDiffVecXd err = p_AQ - p_AP;
  (*y)[0] = err.dot(C_ * err);
}

void PositionCost::DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
                          VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "PositionCost::DoEval() does not work for symbolic variables.");
}

}  // namespace multibody
}  // namespace drake
