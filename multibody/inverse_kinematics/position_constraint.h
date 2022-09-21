#pragma once

#include <memory>
#include <optional>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Constrains the position of a point Q, rigidly attached to a frame B, to be
 * within a bounding box measured and expressed in frame A. Namely
 * p_AQ_lower <= p_AQ <= p_AQ_upper.
 *
 * @ingroup solver_evaluators
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
   * @param plant_context The Context that has been allocated for this
   *   `plant`. We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   * @pre `frameA` and `frameB` must belong to `plant`.
   * @pre p_AQ_lower(i) <= p_AQ_upper(i) for i = 1, 2, 3.
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `plant_context` is nullptr.
   * @pydrake_mkdoc_identifier{double}
   */
  PositionConstraint(const MultibodyPlant<double>* plant,
                     const Frame<double>& frameA,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                     const Frame<double>& frameB,
                     const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                     systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Same as the constructor with the double version
   * (using MultibodyPlant<double> and Context<double>). Except the gradient of
   * the constraint is computed from autodiff.
   * @pydrake_mkdoc_identifier{autodiff}
   */
  PositionConstraint(const MultibodyPlant<AutoDiffXd>* plant,
                     const Frame<AutoDiffXd>& frameA,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                     const Frame<AutoDiffXd>& frameB,
                     const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                     systems::Context<AutoDiffXd>* plant_context);

  /**
   * Overloaded constructor. Except that the constructor takes in a frame A̅ and
   * a pose X_AAbar between the frame A and A̅. We will constrain the position
   * of point Q expressed in the frame A to lie within a bounding box of A.
   * @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   *   should be alive during the lifetime of this constraint.
   * @param frameAbar The frame A̅ in which point Q's position is measured.
   * @param X_AbarA relative transform between the frame A̅ and A. If empty,
   * then we use identity transform.
   * @param p_AQ_lower The lower bound on the position of point Q, measured and
   *   expressed in frame A.
   * @param p_AQ_upper The upper bound on the position of point Q, measured and
   *   expressed in frame A.
   * @param frameB The frame to which point Q is rigidly attached.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   *   measured and expressed in frame B.
   * @param plant_context The Context that has been allocated for this
   *   `plant`. We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   * @pre `frameA` and `frameB` must belong to `plant`.
   * @pre p_AQ_lower(i) <= p_AQ_upper(i) for i = 1, 2, 3.
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `plant_context` is nullptr.
   * @pydrake_mkdoc_identifier{double_Abar}
   */
  PositionConstraint(const MultibodyPlant<double>* plant,
                     const Frame<double>& frameAbar,
                     const std::optional<math::RigidTransformd>& X_AbarA,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                     const Frame<double>& frameB,
                     const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                     systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Same as the constructor with the double version
   * (using MultibodyPlant<double> and Context<double>). Except the gradient of
   * the constraint is computed from autodiff.
   * @pydrake_mkdoc_identifier{autodiff_Abar}
   */
  PositionConstraint(const MultibodyPlant<AutoDiffXd>* plant,
                     const Frame<AutoDiffXd>& frameAbar,
                     const std::optional<math::RigidTransformd>& X_AbarA,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                     const Frame<AutoDiffXd>& frameB,
                     const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                     systems::Context<AutoDiffXd>* plant_context);

  ~PositionConstraint() override {}

  using Constraint::set_bounds;
  using Constraint::UpdateLowerBound;
  using Constraint::UpdateUpperBound;

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

  bool use_autodiff() const { return plant_autodiff_; }

  const MultibodyPlant<double>* const plant_double_;
  const FrameIndex frameAbar_index_;
  math::RigidTransformd X_AAbar_;
  const FrameIndex frameB_index_;
  const Eigen::Vector3d p_BQ_;
  systems::Context<double>* const context_double_;

  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* const context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
