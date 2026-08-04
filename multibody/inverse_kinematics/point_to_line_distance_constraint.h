#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 Constrain that the distance between a point P on frame B1 and another line L on
 frame B2 is within a range [distance_lower, distance_upper].

 @ingroup solver_evaluators
 */
class PointToLineDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointToLineDistanceConstraint);

  /**
   Constrain the distance between a point P attached to frame_point (denoted as
   B1) and the line L attached to frame_line (denoted as B2) is within the range
   [distance_lower, distance_upper].

   Mathematically, we impose the constraint
   distance_lower² <= distance(P, L)² <= distance_upper².
   We impose the constraint on the distance square instead of distance
   directly, because the gradient of distance is not well defined at
   distance=0; on the other hand, the gradient of the distance square is well
   defined everywhere.

   We parameterize the line using a point Q on the line, and a directional
   vector n along the line.

   @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   must be alive during the lifetime of this constraint.
   @param frame_point The frame B1 to which the point P is attached.
   @param p_B1P The position of point P measured and expressed in B1.
   @param frame_line The frame B2 to which the line is attached.
   @param p_B2Q Q is a point on the line, p_B2Q is the position of this point Q
   measured and expressed in B2.
   @param n_B2 n is the directional vector of the line, n_B2 is this vector
   measured and expressed in B2.
   @param distance_lower The lower bound on the distance, must be
   non-negative.
   @param distance_upper The upper bound on the distance, must be
   non-negative.
   @param plant_context The Context that has been allocated for this
     `plant`. We will update the context when evaluating the constraint.
     `plant_context` must be alive during the lifetime of this constraint.
   @pydrake_mkdoc_identifier{double}
   */
  PointToLineDistanceConstraint(const MultibodyPlant<double>* plant,
                                const Frame<double>& frame_point,
                                const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
                                const Frame<double>& frame_line,
                                const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
                                const Eigen::Ref<const Eigen::Vector3d>& n_B2,
                                double distance_lower, double distance_upper,
                                systems::Context<double>* plant_context);

  /**
   Overloaded constructor. Same as the constructor with the double version
   (using MultibodyPlant<double> and Context<double>), except the gradient of
   the constraint is computed from autodiff.
   @pydrake_mkdoc_identifier{autodiff}
   */
  PointToLineDistanceConstraint(const MultibodyPlant<AutoDiffXd>* plant,
                                const Frame<AutoDiffXd>& frame_point,
                                const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
                                const Frame<AutoDiffXd>& frame_line,
                                const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
                                const Eigen::Ref<const Eigen::Vector3d>& n_B2,
                                double distance_lower, double distance_upper,
                                systems::Context<AutoDiffXd>* plant_context);

  ~PointToLineDistanceConstraint() override;

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PointToLineDistanceConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  bool use_autodiff() const { return plant_autodiff_; }

  const MultibodyPlant<double>* const plant_double_;
  const FrameIndex frame_point_index_;
  const FrameIndex frame_line_index_;
  const Eigen::Vector3d p_B1P_;
  const Eigen::Vector3d p_B2Q_;
  const Eigen::Vector3d n_B2_normalized_;
  // project_matrix = I - nnᵀ where n = n_B2_normalized_
  const Eigen::Matrix3d project_matrix_;
  systems::Context<double>* const context_double_;

  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* const context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
