#include "drake/multibody/optimization/static_equilibrium_solver.h"

#include "drake/multibody/optimization/static_equilibrium_constraint.h"
#include "drake/multibody/optimization/static_friction_cone_complementary_constraint.h"

namespace drake {
namespace multibody {
StaticEquilibriumSolver::StaticEquilibriumSolver(
    const MultibodyPlant<AutoDiffXd>* plant,
    systems::Context<AutoDiffXd>* context,
    const std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>&
        ignored_collision_pairs)
    : plant_(plant),
      context_(context),
      owned_prog_{new solvers::MathematicalProgram()},
      prog_{owned_prog_.get()},
      q_vars_{prog_->NewContinuousVariables(plant->num_positions())},
      u_vars_{prog_->NewContinuousVariables(plant->num_actuated_dofs())} {
  // declare the contact force as decision variables, and pair each contact
  // force decision variable with the evaluator to compute the contact wrench.
  const auto& query_port = plant_->get_geometry_query_input_port();
  if (!query_port.HasValue(*context_)) {
    throw std::invalid_argument(
        "StaticEquilibriumConstraint: Cannot get a valid "
        "geometry::QueryObject. Please refer to AddMultibodyPlantSceneGraph "
        "on connecting MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<AutoDiffXd>>(*context_);
  const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
      query_object.inspector();
  const std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>
      collision_candidate_pairs = inspector.GetCollisionCandidates();
  contact_wrench_evaluators_and_lambda_.reserve(
      collision_candidate_pairs.size());
  for (const auto& collision_candidate_pair : collision_candidate_pairs) {
    if (ignored_collision_pairs.count(collision_candidate_pair) == 0) {
      auto contact_wrench_evaluator =
          std::make_shared<ContactWrenchFromForceInWorldFrameEvaluator>(
              plant_, context_, collision_candidate_pair);
      auto lambda_var = prog_->NewContinuousVariables<3>();
      contact_wrench_evaluators_and_lambda_.emplace_back(
          contact_wrench_evaluator, lambda_var);
    }
  }

  prog_->AddConstraint(StaticEquilibriumConstraint::MakeBinding(
      plant_, context_, contact_wrench_evaluators_and_lambda_, q_vars_,
      u_vars_));

  const double complementary_tol = 1E-3;
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda_) {
    AddStaticFrictionConeComplementaryConstraint(
        contact_wrench_evaluator_and_lambda.first.get(), complementary_tol,
        q_vars_, contact_wrench_evaluator_and_lambda.second, prog_);
  }
}
}  // namespace multibody
}  // namespace drake
