#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Constrains the position of a point Q, rigidly attached to a frame B, to be
 * within a bounding box measured and expressed in frame A. Namely
 * p_AQ_lower <= p_AQ <= p_AQ_upper.
 */
class PositionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionConstraint)

  /**
   * Constructs PositionConstraint object.
   * @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   *   should be alive during the lifetime of this constraint.
   * @param frameA The frame in which point Q's position is measured.
   * @param p_AQ_lower The lower bound on the position of point Q, measured and
   *   expressed in frame A.
   * @param p_AQ_upper The upper bound on the position of point Q, measured and
   *   expressed in frame A.
   * @param frameB The frame to which point Q is rigidly attached.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   *   measured and expressed in frame B.
   * @param context The Context that has been allocated for this `plant`. We
   *   will update the context when evaluating the constraint. `context` should
   *   be alive during the lifetime of this constraint.
   * @pre `frameA` and `frameB` must belong to `plant`.
   * @pre p_AQ_lower(i) <= p_AQ_upper(i) for i = 1, 2, 3.
   * @throws std::invalid_argument if `plant` is nullptr.
   * @throws std::invalid_argument if `context` is nullptr.
   */
  PositionConstraint(const MultibodyPlant<double>* const plant,
                     const Frame<double>& frameA,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                     const Frame<double>& frameB,
                     const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                     systems::Context<double>* context);

  ~PositionConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PositionConstraint::DoEval() does not work for symbolic variables.");
  }

  const MultibodyPlant<double>& plant_;
  const FrameIndex frameA_index_;
  const FrameIndex frameB_index_;
  const Eigen::Vector3d p_BQ_;
  systems::Context<double>* const context_;
};
}  // namespace multibody
}  // namespace drake
