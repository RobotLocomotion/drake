#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Constrain that the distance between a point P1 on frame B1 and another point
 * P2 on frame B2 is within a range [distance_lower, distance_upper].
 */
class PointToPointDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointToPointDistanceConstraint)

  /**
   * Constrain that the distance between a point P1 attached to frame B1 and
   * another point P2 attached to frame B2 is within the range [distance_lower,
   * distance_upper].
   * Mathematically, we impose the constraint
   * distance_lower² <= distance(P1, P2)² <= distance_upper².
   * We impose the constraint on the distance square instead of distance
   * directly, because the gradient of distance is not well defined at
   * distance=0, the the gradient of the distance square is well defined
   * everywhere.
   * @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   * should be alive during the lifetime of this constraint.
   * @param frame1 The frame in which P1 is attached to.
   * @param p_B1P1 The position of P1 measured and expressed in B1.
   * @param frame2 The frame in which P2 is attached to.
   * @param p_B2P2 The position of P2 measured and expressed in B2.
   * @param distance_lower The lower bound on the distance, must be
   * non-negative.
   * @param distance_upper The upper bound on the distance, must be
   * non-negative.
   * @param plant_context The Context that has been allocated for this
   *   `plant`. We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   */
  PointToPointDistanceConstraint(
      const MultibodyPlant<double>* plant, const Frame<double>& frame1,
      const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
      const Frame<double>& frame2,
      const Eigen::Ref<const Eigen::Vector3d>& p_B2P2, double distance_lower,
      double distance_upper, systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Same as the constructor with the double version
   * (using MultibodyPlant<double> and Context<double>), except the gradient of
   * the constraint is computed from autodiff.
   * @exclude_from_pydrake_mkdoc{Suppressed due to ambiguity in mkdoc.
   * Documentation string is manually recreated in Python.}
   */
  PointToPointDistanceConstraint(
      const MultibodyPlant<AutoDiffXd>* plant, const Frame<AutoDiffXd>& frame1,
      const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
      const Frame<AutoDiffXd>& frame2,
      const Eigen::Ref<const Eigen::Vector3d>& p_B2P2, double distance_lower,
      double distance_upper, systems::Context<AutoDiffXd>* plant_context);

  ~PointToPointDistanceConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PointToPointDistanceConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  bool use_autodiff() const { return plant_autodiff_; }

  const MultibodyPlant<double>* const plant_double_;
  const FrameIndex frame1_index_;
  const FrameIndex frame2_index_;
  const Eigen::Vector3d p_B1P1_;
  const Eigen::Vector3d p_B2P2_;
  systems::Context<double>* const context_double_;

  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* const context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
