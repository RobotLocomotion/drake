#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
enum class MinimumDistancePenaltyType {
  kSmoothedHinge,  ///< A linear hinge loss, smoothed with a quadratic loss near
                   ///< the origin. The formulation is in equation (6) of Loss
                   ///< Functions for Preference Levels: Regression with
                   ///< Discrete Ordered Lables.
                   ///< The penalty is
                   ///< γ(x) = 0 if x ≥ 0;
                   ///< γ(x) = x²/2 if -1 < x < 0;
                   ///< γ(x) = -0.5 - x if x ≤ -1
  kExponential,    ///< A smoothed penalty function (differentiable everywhere)
                   ///< that uses exponential functions. The fomulation is
                   ///< described in section II.C of Whole-body Motion Planning
                   ///< with Centroidal Dynamics and Full Kinematics.
                   ///< The penalty is
                   ///< γ(x) = 0 if x ≥ 0;
                   ///< γ(x) = -x exp(1/x) if x < 0
};

/**
 * Constrain that the pairwise distance between objects should be no
 * smaller than a positive threshold. We consider the distance between pairs of
 * 1. Anchored (static) object and a dynamic object.
 * 2. A dynamic object and another dynamic object, if one is not the parent
 * link of the other.
 * The formulation of the constraint is
 * ∑ γ(φᵢ/dₘᵢₙ - 1) = 0
 * where φᵢ is the signed distance of the i'th pair, dₘᵢₙ is the minimum
 * allowable distance, and γ is a penalty function.
 */
class MinimumDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceConstraint)

  /**
   * @param plant The robot on which the inverse kinematics problem will be
   * solved. This plant has to have registered its geometry with a SceneGraph
   * object. @throws invalid_argument if the plant has not registered its
   * geometry.
   * @param minimum_distance The minimum value of the signed distance between
   * any admissible pairs of objects.
   * @param penalty_type The penalty function formulation.
   * @pre The MultibodyPlant passed in the constructor of InverseKinematics has
   * registered its geometry with a SceneGraph object already.
   * @pre minimum_distance > 0.
   * @throw invalid_argument if the geometry hasn't been registered.
   */
  MinimumDistanceConstraint(
      const multibody::MultibodyPlant<double>* const plant,
      double minimum_distance, systems::Context<double>* plant_context,
      MinimumDistancePenaltyType penalty_type =
          MinimumDistancePenaltyType::kSmoothedHinge);

  ~MinimumDistanceConstraint() override {}

  /** Getter for the minimum distance. */
  double minimum_distance() const { return minimum_distance_; }

  /** Getter for the penalyt function type. */
  MinimumDistancePenaltyType penalty_type() const { return penalty_type_; }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "MinimumDistanceConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  const multibody::MultibodyPlant<double>& plant_;
  const double minimum_distance_;
  systems::Context<double>* const plant_context_;
  MinimumDistancePenaltyType penalty_type_;
};
}  // namespace multibody
}  // namespace drake
