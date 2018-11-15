#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * Constrain that the pairwise distance between objects should be no
 * smaller than a positive threshold. We consider the distance between pairs of
 * 1. Anchored (static) object and a dynamic object.
 * 2. A dynamic object and another dynamic object, if one is not the parent
 * link of the other.
 */
class MinimumDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceConstraint)

  /**
   * @param plant The robot on which the inverse kinematics problem will be
   * solved. If this plant has registered its geometry with a SceneGraph object,
   * then the user can impose collision related constraint (like
   * AddMinimumDistanceConstraint).
   * @param minimum_distance The minimum value of the signed distance between
   * any admissible pairs of objects.
   * @pre The MultibodyPlant passed in the constructor of InverseKinematics has
   * registered its geometry with a SceneGraph object already. @throw
   * invalid_argument if the geometry hasn't been registered.
   * @pre minimum_distance > 0.
   * The formulation of the constraint is
   * ∑ γ(φᵢ/dₘᵢₙ - 1) = 0
   * where φᵢ is the signed distance of the i'th pair, dₘᵢₙ is the minimum
   * allowable distance, and γ is a penalizing function defined as
   * γ(x) = 0 if x ≥ 0
   * γ(x) = -x exp(1/x) if x < 0
   * This formulation is described in section II.C of Whole-body Motion planning
   * with Centroidal Dynamics and Full Kinematics by Hongkai Dai, Andres
   * Valenzuela and Russ Tedrake, 2014 IEEE-RAS International Conference on
   * Humanoid Robots.
   */
  MinimumDistanceConstraint(
      const multibody::multibody_plant::MultibodyPlant<AutoDiffXd>& plant,
      double minimum_distance, systems::Context<AutoDiffXd>* plant_context);

  ~MinimumDistanceConstraint() override {}

  /** Getter for the minimum distance. */
  double minimum_distance() const { return minimum_distance_; }

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

  const multibody::multibody_plant::MultibodyPlant<AutoDiffXd>& plant_;
  const double minimum_distance_;
  systems::Context<AutoDiffXd>* const plant_context_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
