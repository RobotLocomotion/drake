#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Computes the penalty function γ(x) and its derivatives dγ(x)/dx, where x is
 * the scaled (and shifted) signed distance (x = distance / distance_threshold
 * - 1). This function is used by MinimumDistanceConstraint, in which we impose
 * the constraint that the pairwise distance are all no smaller than a distance
 * threshold. We do this with the constraint
 * ∑ᵢ γ(dᵢ / distance_threshold - 1) = 0
 * where dᵢ is the signed distance between the i'th pair of geometries.
 */
using MinimumDistancePenaltyFunction =
    std::function<void(double x, double* penalty, double* dpenalty_dx)>;

/// A hinge loss function smoothed by exponential function. This loss
/// function is differentiable everywhere. The fomulation is described in
/// section II.C of [2]
/// The penalty is
/// <pre>
///        ⎧ 0            if x ≥ 0
/// γ(x) = ⎨
///        ⎩  -x exp(1/x) if x < 0.
/// </pre>
/// [2] "Whole-body Motion Planning with Centroidal Dynamics and Full
/// Kinematics" by Hongkai Dai, Andres Valenzuela and Russ Tedrake, IEEE-RAS
/// International Conference on Humanoid Robots, 2014.
void ExponentiallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx);

/// A linear hinge loss, smoothed with a quadratic loss near the origin. The
/// formulation is in equation (6) of [1].
/// The penalty is
/// <pre>
///        ⎧  0        if x ≥ 0
/// γ(x) = ⎨  x²/2     if -1 < x < 0
///        ⎩  -0.5 - x if x ≤ -1.
/// </pre>
/// [1] "Loss Functions for Preference Levels: Regression with Discrete
/// Ordered Labels." by Jason Rennie and Nathan Srebro, Proceedings of IJCAI
/// multidisciplinary workshop on Advances in preference handling.
void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx);

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
      MinimumDistancePenaltyFunction penalty_function =
          QuadraticallySmoothedHingeLoss);

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

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  const multibody::MultibodyPlant<double>& plant_;
  const double minimum_distance_;
  systems::Context<double>* const plant_context_;
  MinimumDistancePenaltyFunction penalty_function_;
};
}  // namespace multibody
}  // namespace drake
