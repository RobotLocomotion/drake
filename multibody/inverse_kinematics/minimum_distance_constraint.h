#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Constrain that the pairwise distance between objects should be no
 * smaller than a positive threshold. We consider the distance between pairs of
 * 1. Anchored (static) object and a dynamic object.
 * 2. A dynamic object and another dynamic object, if one is not the parent
 * link of the other.
 * The formulation of the constraint is
 * ∑ γ(φᵢ/dₘᵢₙ - 1) = 0
 * where φᵢ is the signed distance of the i'th pair, dₘᵢₙ is the minimal
 * allowable distance, and γ is a penalizing function defined as
 * γ(x) = 0 if x ≥ 0
 * γ(x) = -x exp(1/x) if x < 0
 * This formulation is described in section II.C of Whole-body Motion Planning
 * with Centroidal Dynamics and Full Kinematics by Hongkai Dai, Andres
 * Valenzuela and Russ Tedrake, 2014 IEEE-RAS International Conference on
 * Humanoid Robots.
 */
class MinimumDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceConstraint)

  /**
   * @param plant The robot on which the inverse kinematics problem will be
   * solved. This plant has to have registered its geometry with a SceneGraph
   * object. @throws invalid_argument if the plant has not registered its
   * geometry.
   * @param minimal_distance The minimal value of the signed distance between
   * any admissible pairs of objects.
   * @pre The MultibodyPlant passed in the constructor of InverseKinematics has
   * registered its geometry with a SceneGraph object already.
   * @pre minimal_distance > 0.
   * @throw invalid_argument if the geometry hasn't been registered.
   */
  MinimumDistanceConstraint(
      const multibody::MultibodyPlant<double>* const plant,
      double minimal_distance, systems::Context<double>* plant_context);

  ~MinimumDistanceConstraint() override {}

  /** Getter for the minimal distance. */
  double minimal_distance() const { return minimal_distance_; }

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
  const double minimal_distance_;
  systems::Context<double>* const plant_context_;
};
}  // namespace multibody
}  // namespace drake
