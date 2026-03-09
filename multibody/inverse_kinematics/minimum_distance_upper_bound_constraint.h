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
/** Constrain min(d) <= ub, namely at least one signed distance between a
candidate pairs of geometries (according to the logic of
SceneGraphInspector::GetCollisionCandidates()) to be no larger than a specified
ub.
This constraint should be bound to decision variables corresponding to the
configuration vector, q, of the associated MultibodyPlant.

The formulation of the constraint is

    SmoothUnderMax( φ((dᵢ(q) - d_influence)/(d_influence - ub)) / φ(-1) ) ≥ 1

where dᵢ(q) is the signed distance of the i-th pair, ub is the upper bound of
the minimum distance, d_influence is the "influence distance" (the distance
below which a pair of geometries influences the constraint), φ is a
solvers::MinimumValuePenaltyFunction. SmoothUnderMax(d)
is smooth under approximation of max(d). We require
that ub < d_influence. The input scaling (dᵢ(q) - d_influence)/(d_influence -
ub) ensures that at the boundary of the feasible set (when dᵢ(q) == ub), we
evaluate the penalty function at -1, where it is required to have a non-zero
gradient.

@ingroup solver_evaluators
*/
class MinimumDistanceUpperBoundConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceUpperBoundConstraint);

  /** Constructs a MinimumDistanceUpperBoundConstraint.
  @param plant The multibody system on which the constraint will be evaluated.
  @param bound `ub` in the class documentation. The upper bound
  minimum signed distance between any candidate pair of geometries.
  @param plant_context The context of `plant`. The context should be obtained as
  a subsystem context from the diagram context, where the diagram (that contains
  both the MultibodyPlant and SceneGraph) creates the diagram context.
  `plant_context` cannot be a nullptr. `plant_context` must outlive this
  constraint. An example code of getting the plant context is
  @code{cc}
  auto diagram_context = diagram.CreateDefaultContext();
  auto plant_context = plant.GetMyMutableContextFromRoot(diagram_context.get());
  @endcode
  @param penalty_function The penalty function formulation.
  @default QuadraticallySmoothedHinge
  @param influence_distance_offset The difference (in meters) between the
  influence distance, d_influence, and the minimum distance_upper, ub (see class
  documentation), namely influence_distance = bound + influence_distance_offset.
  This value must be finite and strictly positive, as it is used to scale the
  signed distances between pairs of geometries. Larger value might increase the
  possibility of finding a solution through gradient based nonlinear
  optimization. This is because a geometry pair with distance larger than
  `influence_distance` is ignored, so is its gradient; hence the gradient-based
  optimizer doesn't know to actively reduce the distance between that pair. We
  strongly suggest to use a different (and larger) `influence_distance_offset`
  as the one used in MinimumValueLowerBoundConstraint.
  @throws std::exception if `plant` has not registered its geometry with
  a SceneGraph object.
  @throws std::exception if influence_distance_offset = ∞.
  @throws std::exception if influence_distance_offset ≤ 0.
  @pydrake_mkdoc_identifier{double_mbp}
  */
  MinimumDistanceUpperBoundConstraint(
      const multibody::MultibodyPlant<double>* const plant, double bound,
      systems::Context<double>* plant_context, double influence_distance_offset,
      solvers::MinimumValuePenaltyFunction penalty_function = {});

  /**
  Overloaded constructor.
  Constructs the constraint using MultibodyPlant<AutoDiffXd>.
  @pydrake_mkdoc_identifier{autodiff_mbp}
  */
  MinimumDistanceUpperBoundConstraint(
      const multibody::MultibodyPlant<AutoDiffXd>* const plant, double bound,
      systems::Context<AutoDiffXd>* plant_context,
      double influence_value_offset,
      solvers::MinimumValuePenaltyFunction penalty_function = {});

  /** Overloaded constructor.
  Constructs the constraint with CollisionChecker instead of MultibodyPlant.
  @param collision_checker collision_checker must outlive this constraint.
  @param collision_checker_context The context for the collision checker. See
  CollisionChecker class for more details.
  @pydrake_mkdoc_identifier{collision_checker}
  */
  MinimumDistanceUpperBoundConstraint(
      const planning::CollisionChecker* collision_checker, double bound,
      planning::CollisionCheckerContext* collision_checker_context,
      double influence_distance_offset,
      solvers::MinimumValuePenaltyFunction penalty_function = {});

  ~MinimumDistanceUpperBoundConstraint() override;

  /** Getter for the upper bound of the minimum distance. */
  double distance_bound() const {
    return minimum_value_constraint_->minimum_value_upper();
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
        "MinimumDistanceUpperBoundConstraint::DoEval() does not work for "
        "symbolic variables.");
  }

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  void CheckBounds(double bound, double influence_distance) const;

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
  std::unique_ptr<solvers::MinimumValueUpperBoundConstraint>
      minimum_value_constraint_{};
  const multibody::MultibodyPlant<AutoDiffXd>* const plant_autodiff_{};
  systems::Context<AutoDiffXd>* const plant_context_autodiff_{};

  const planning::CollisionChecker* collision_checker_{};
  planning::CollisionCheckerContext* collision_checker_context_{};
};
}  // namespace multibody
}  // namespace drake
