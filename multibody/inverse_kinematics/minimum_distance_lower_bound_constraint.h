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
solvers::MinimumValuePenaltyFunction. SmoothOverMax(d)
is smooth over approximation of max(d). We require
that lb < d_influence. The input scaling (dᵢ(q) - d_influence)/(d_influence -
lb) ensures that at the boundary of the feasible set (when dᵢ(q) == lb), we
evaluate the penalty function at -1, where it is required to have a non-zero
gradient.

@ingroup solver_evaluators
*/
class MinimumDistanceLowerBoundConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceLowerBoundConstraint);

  /** Constructs a MinimumDistanceLowerBoundConstraint.
  @param plant The multibody system on which the constraint will be evaluated.
  `plant` cannot be a nullptr. `plant` must outlive this constraint.
  @param bound The minimum allowed value, lb, of the signed
  distance between any candidate pair of geometries.
  @param penalty_function The penalty function formulation.
  @default QuadraticallySmoothedHinge
  @param plant_context The context of `plant`. The context should be obtained as
  a subsystem context from the diagram context, where the diagram (that contains
  both the MultibodyPlant and SceneGraph) creates the diagram context.
  `plant_context` cannot be a nullptr. `plant_context` must outlive this
  constraint. An example code of getting the plant context is
  @code{cc}
  auto diagram_context = diagram.CreateDefaultContext();
  auto plant_context = plant.GetMyMutableContextFromRoot(diagram_context.get());
  @endcode
  @param influence_distance_offset The difference (in meters) between the
  influence distance, d_influence, and the minimum distance, lb (see class
  documentation), namely influence_distance = bound +
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
  @pydrake_mkdoc_identifier{double_mbp}
  */
  MinimumDistanceLowerBoundConstraint(
      const multibody::MultibodyPlant<double>* const plant, double bound,
      systems::Context<double>* plant_context,
      solvers::MinimumValuePenaltyFunction penalty_function = {},
      double influence_distance_offset = 0.01);

  /**
  Overloaded constructor.
  Constructs the constraint using MultibodyPlant<AutoDiffXd>.
  @pydrake_mkdoc_identifier{autodiff_mbp}
  */
  MinimumDistanceLowerBoundConstraint(
      const multibody::MultibodyPlant<AutoDiffXd>* const plant, double bound,
      systems::Context<AutoDiffXd>* plant_context,
      solvers::MinimumValuePenaltyFunction penalty_function = {},
      double influence_distance_offset = 0.01);

  /** Overloaded constructor.
  Constructs the constraint with CollisionChecker instead of MultibodyPlant.
  @param collision_checker collision_checker must outlive this constraint.
  @param collision_checker_context The context for the collision checker. See
  CollisionChecker class for more details.
  @pydrake_mkdoc_identifier{collision_checker}
  */
  MinimumDistanceLowerBoundConstraint(
      const planning::CollisionChecker* collision_checker, double bound,
      planning::CollisionCheckerContext* collision_checker_context,
      solvers::MinimumValuePenaltyFunction penalty_function = {},
      double influence_distance_offset = 0.01);

  ~MinimumDistanceLowerBoundConstraint() override;

  /** Getter for the lower bound of the minimum distance. */
  double distance_bound() const {
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
                  systems::Context<T>* plant_context, double bound,
                  double influence_distance_offset,
                  const solvers::MinimumValuePenaltyFunction& penalty_function);

  // Overload Initialize with CollisionChecker instead of MultibodyPlant.
  void Initialize(const planning::CollisionChecker& collision_checker,
                  planning::CollisionCheckerContext* collision_checker_context,
                  double bound, double influence_distance_offset,
                  const solvers::MinimumValuePenaltyFunction& penalty_function);

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
