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
   * @param tree The multibody tree on which the constraint is imposed. @p tree
   *   should be alive during the whole lifetime of this constraint.
   * @param frameB Frame B.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   *   measured and expressed in frame B.
   * @param frameA_idx Frame A.
   * @param p_AQ_lower The lower bound on the position of point Q, measured and
   *   expressed in frame A.
   * @param p_AQ_upper The upper bound on the position of point Q, measured and
   *   expressed in frame A.
   * @param context The Context that has been allocated for this @p tree. We
   *   will update the context when evaluating the constraint. @p context should
   *   be alive during the lifetime of this constraint.
   */
  PositionConstraint(const multibody_plant::MultibodyPlant<double>* const plant,
                     const Frame<double>& frameB,
                     const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                     const Frame<double>& frameA,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
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

  const multibody_plant::MultibodyPlant<double>& plant_;
  const Frame<double>& frameB_;
  const Frame<double>& frameA_;
  const Eigen::Vector3d p_BQ_;
  systems::Context<double>* const context_;
};
}  // namespace multibody
}  // namespace drake
