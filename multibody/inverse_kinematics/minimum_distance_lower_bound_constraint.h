#pragma once
#include <memory>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/collision_checker.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/minimum_value_constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/** Computes the penalty function φ(x) and its derivatives dφ(x)/dx. Valid
penalty functions must meet the following criteria:

1.     φ(x) ≥ 0 ∀ x ∈ ℝ.
2. dφ(x)/dx ≤ 0 ∀ x ∈ ℝ.
3.     φ(x) = 0 ∀ x ≥ 0.
4. dφ(x)/dx < 0 ∀ x < 0. */
using MinimumDistancePenaltyFunction = solvers::MinimumValuePenaltyFunction;

/** A hinge loss function smoothed by exponential function. This loss
function is differentiable everywhere. The formulation is described in
section II.C of [2].
The penalty is
<pre>
       ⎧ 0            if x ≥ 0
φ(x) = ⎨
       ⎩  -x exp(1/x) if x < 0.
</pre>
[2] "Whole-body Motion Planning with Centroidal Dynamics and Full
Kinematics" by Hongkai Dai, Andres Valenzuela and Russ Tedrake, IEEE-RAS
International Conference on Humanoid Robots, 2014. */
using solvers::ExponentiallySmoothedHingeLoss;

/** A linear hinge loss, smoothed with a quadratic loss near the origin. The
formulation is in equation (6) of [1].
The penalty is
<pre>
       ⎧  0        if x ≥ 0
φ(x) = ⎨  x²/2     if -1 < x < 0
       ⎩  -0.5 - x if x ≤ -1.
</pre>
[1] "Loss Functions for Preference Levels: Regression with Discrete Ordered
Labels." by Jason Rennie and Nathan Srebro, Proceedings of IJCAI
multidisciplinary workshop on Advances in Preference Handling. */
using solvers::QuadraticallySmoothedHingeLoss;
/** Constrain min(d) >= lb, namely the signed distance between all
candidate pairs of geometries (according to the logic of
SceneGraphInspector::GetCollisionCandidates()) to be no smaller than a specified
minimum distance lb.
This constraint should be bound to decision variables corresponding to the
configuration vector, q, of the associated MultibodyPlant.

The formulation of the constraint is

    SmoothOverMax( φ((dᵢ(q) - d_influence)/(d_influence - lb)) / φ(-1) ) ≤ 1

where dᵢ(q) is the signed distance of the i-th pair, lb is the minimum
allowable distance, d_influence is the "influence distance" (the distance below
which a pair of geometries influences the constraint), φ is a
multibody::MinimumDistancePenaltyFunction. SmoothOverMax(d)
is smooth over approximation of max(d). We require
that lb < d_influence. The input scaling (dᵢ(q) - d_influence)/(d_influence -
lb) ensures that at the boundary of the feasible set (when dᵢ(q) == lb), we
evaluate the penalty function at -1, where it is required to have a non-zero
gradient.

@ingroup solver_evaluators
*/
class MinimumDistanceLowerBoundConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceLowerBoundConstraint)

  /** Constructs a MinimumDistanceLowerBoundConstraint.
  @param plant The multibody system on which the constraint will be evaluated.
  @param minimum_distance_lower The minimum allowed value, lb, of the signed
  distance between any candidate pair of geometries.
  @param penalty_function The penalty function formulation.
  @default QuadraticallySmoothedHinge
  @param influence_distance_offset The difference (in meters) between the
  influence distance, d_influence, and the minimum distance, lb (see class
  documentation), namely influence_distance = minimum_distance_lower +
  influence_distance_offset. This value must be finite and strictly positive, as
  it is used to scale the signed distances between pairs of geometries. Smaller
  values may improve performance, as fewer pairs of geometries need to be
  considered in each constraint evaluation. @default 0.01 meter.
  The chosen influence_distance_offset can significantly affect the runtime and
  optimization performance of using this constraint. Larger values result in
  more expensive collision checks (since more potential collision candidates
  must be considered) and may result in worse optimization performance (the
  optimizer may not be able to find a configuration that satisfies the
  constraint). In work at TRI, we have used much lower values (e.g. 1e-6) for
  influence_distance_offset with good results.
  @throws std::exception if `plant` has not registered its geometry with
  a SceneGraph object.
  @throws std::exception if influence_distance_offset = ∞.
  @throws std::exception if influence_distance_offset ≤ 0.
  @pydrake_mkdoc_identifier{double_no_upper_bound}
  */
  MinimumDistanceLowerBoundConstraint(
      const multibody::MultibodyPlant<double>* const plant,
      double minimum_distance_lower, systems::Context<double>* plant_context,
      MinimumDistancePenaltyFunction penalty_function = {},
      double influence_distance_offset = 0.01);

  /**
  Overloaded constructor.
  Constructs the constraint using MultibodyPlant<AutoDiffXd>.
  @pydrake_mkdoc_identifier{autodiff_no_upper_bound}
  */
  MinimumDistanceLowerBoundConstraint(
      const multibody::MultibodyPlant<AutoDiffXd>* const plant,
      double minimum_distance_lower,
      systems::Context<AutoDiffXd>* plant_context,
      MinimumDistancePenaltyFunction penalty_function = {},
      double influence_distance_offset = 0.01);

  /** Overloaded constructor.
  Constructs the constraint with CollisionChecker instead of MultibodyPlant.
  @param collision_checker collision_checker must outlive this constraint.
  @param collision_checker_context The context for the collision checker. See
  CollisionChecker class for more details.
  @pydrake_mkdoc_identifier{collision_checker_no_upper_bound}
  */
  MinimumDistanceLowerBoundConstraint(
      const planning::CollisionChecker* collision_checker,
      double minimum_distance_lower,
      planning::CollisionCheckerContext* collision_checker_context,
      MinimumDistancePenaltyFunction penalty_function = {},
      double influence_distance_offset = 0.01);

  ~MinimumDistanceLowerBoundConstraint() override {}

  /** Getter for the lower bound of the minimum distance. */
  double minimum_distance_lower() const {
    return minimum_value_constraint_->minimum_value_lower();
  }

  /** Getter for the influence distance. */
  double influence_distance() const {
    return minimum_value_constraint_->influence_value();
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "MinimumDistanceLowerBoundConstraint::DoEval() does not work for "
        "symbolic variables.");
  }

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  void CheckBounds(double minimum_distance_upper,
                   double influence_distance) const;

  template <typename T>
  void Initialize(const MultibodyPlant<T>& plant,
                  systems::Context<T>* plant_context,
                  double minimum_distance_lower,
                  double influence_distance_offset,
                  const MinimumDistancePenaltyFunction& penalty_function);

  // Overload Initialize with CollisionChecker instead of MultibodyPlant.
  void Initialize(const planning::CollisionChecker& collision_checker,
                  planning::CollisionCheckerContext* collision_checker_context,
                  double minimum_distance_lower,
                  double influence_distance_offset,
                  const MinimumDistancePenaltyFunction& penalty_function);

  const multibody::MultibodyPlant<double>* const plant_double_{};
  systems::Context<double>* const plant_context_double_{};
  std::unique_ptr<solvers::MinimumValueLowerBoundConstraint>
      minimum_value_constraint_{};
  const multibody::MultibodyPlant<AutoDiffXd>* const plant_autodiff_{};
  systems::Context<AutoDiffXd>* const plant_context_autodiff_{};

  const planning::CollisionChecker* collision_checker_{};
  planning::CollisionCheckerContext* collision_checker_context_{};
};

}  // namespace multibody
}  // namespace drake
