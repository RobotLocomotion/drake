#pragma once

#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/cost.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Implements a cost of the form (p_AP - p_AQ)ᵀ C (p_AP - p_AQ), where point P
 * is specified relative to frame A and point Q is specified relative to frame
 * B, and the cost is evaluated in frame A.
 *
 * @ingroup solver_evaluators
 */
class PositionCost : public solvers::Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionCost);

  /**
   * Constructs PositionCost object.
   * @param plant The MultibodyPlant on which the cost is implemented. `plant`
   *   should be alive during the lifetime of this cost.
   * @param frameA The frame in which point P's position is measured.
   * @param p_AP The point P.
   * @param frameB The frame in which point Q's position is measured.
   * @param p_BQ The point Q.
   * @param C A 3x3 matrix representing the cost in quadratic form.
   * @param plant_context A context for the `plant`.
   *
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `plant_context` is nullptr.
   * @pydrake_mkdoc_identifier{double}
   */
  PositionCost(const MultibodyPlant<double>* plant, const Frame<double>& frameA,
               const Eigen::Ref<const Eigen::Vector3d>& p_AP,
               const Frame<double>& frameB,
               const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
               const Eigen::Ref<const Eigen::Matrix3d>& C,
               systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Same as the constructor with the double version
   * (using MultibodyPlant<double> and Context<double>). Except the gradient of
   * the cost is computed from autodiff.
   * @pydrake_mkdoc_identifier{autodiff}
   */
  PositionCost(const MultibodyPlant<AutoDiffXd>* plant,
               const Frame<AutoDiffXd>& frameA,
               const Eigen::Ref<const Eigen::Vector3d>& p_AP,
               const Frame<AutoDiffXd>& frameB,
               const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
               const Eigen::Ref<const Eigen::Matrix3d>& C,
               systems::Context<AutoDiffXd>* plant_context);

  ~PositionCost() override;

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override;

  const PositionConstraint constraint_;
  const Eigen::Matrix3d C_;
};
}  // namespace multibody
}  // namespace drake
