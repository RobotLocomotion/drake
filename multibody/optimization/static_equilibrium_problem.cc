#include "drake/multibody/optimization/static_equilibrium_problem.h"

#include "drake/multibody/optimization/static_equilibrium_constraint.h"
#include "drake/multibody/optimization/static_friction_cone_complementary_constraint.h"

namespace drake {
namespace multibody {
StaticEquilibriumProblem::StaticEquilibriumProblem(
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
  static_friction_cone_complementary_nonlinear_constraints_.reserve(
      contact_wrench_evaluators_and_lambda_.size());
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda_) {
    static_friction_cone_complementary_nonlinear_constraints_.push_back(
        AddStaticFrictionConeComplementaryConstraint(
            contact_wrench_evaluator_and_lambda.first.get(), complementary_tol,
            q_vars_, contact_wrench_evaluator_and_lambda.second, prog_));
  }
}

std::vector<ContactWrench> StaticEquilibriumProblem::GetContactWrenchSolution(
    const solvers::MathematicalProgramResult& result) {
  // TODO(hongkai.dai): store the mapping from pair of geometry IDs to the
  // contact wrench evaluator and lambda.
  const auto q_sol = result.GetSolution(q_vars_);
  const auto u_sol = result.GetSolution(u_vars_);
  plant_->SetPositions(context_, q_sol.cast<AutoDiffXd>());
  const auto& query_port = plant_->get_geometry_query_input_port();
  if (!query_port.HasValue(*context_)) {
    throw std::invalid_argument(
        "StaticEquilibriumConstraint: Cannot get a valid "
        "geometry::QueryObject. Please refer to AddMultibodyPlantSceneGraph "
        "on connecting MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<AutoDiffXd>>(*context_);
  const std::vector<geometry::SignedDistancePair<AutoDiffXd>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();
  const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
      query_object.inspector();
  std::vector<ContactWrench> contact_wrench_sol;
  contact_wrench_sol.reserve(contact_wrench_evaluators_and_lambda_.size());
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda_) {
    const auto& contact_wrench_evaluator =
        contact_wrench_evaluator_and_lambda.first;
    // Compute the contact wrench
    const auto lambda_sol =
        result.GetSolution(contact_wrench_evaluator_and_lambda.second);
    Eigen::VectorXd F_Cb_W;
    contact_wrench_evaluator->Eval(
        contact_wrench_evaluator->ComposeVariableValues(q_sol, lambda_sol),
        &F_Cb_W);
    // Compute p_WCb, where the contact wrench is applied.
    // TODO(hongkai.dai) compute the signed distance between THIS pair of
    // contact, rather than all pairs of contact.
    Vector3<AutoDiffXd> p_WCb;
    const auto& contact_pair = contact_wrench_evaluator->geometry_id_pair();
    BodyIndex bodyA_index, bodyB_index;
    for (const auto& signed_distance_pair : signed_distance_pairs) {
      if (signed_distance_pair.id_A == contact_pair.first &&
          signed_distance_pair.id_B == contact_pair.second) {
        const geometry::FrameId frame_A_id =
            inspector.GetFrameId(signed_distance_pair.id_A);
        const geometry::FrameId frame_B_id =
            inspector.GetFrameId(signed_distance_pair.id_B);
        const Frame<AutoDiffXd>& frameA =
            plant_->GetBodyFromFrameId(frame_A_id)->body_frame();
        const Frame<AutoDiffXd>& frameB =
            plant_->GetBodyFromFrameId(frame_B_id)->body_frame();
        bodyA_index = frameA.body().index();
        bodyB_index = frameB.body().index();
        // Define Body B's frame as B, the geometry attached to body B has frame
        // Gb, and the witness point on geometry Ga is Cb.
        const auto& X_BGb = inspector.X_FG(signed_distance_pair.id_B);
        const auto& p_GbCb = signed_distance_pair.p_BCb;
        const Vector3<AutoDiffXd> p_BCb = X_BGb * p_GbCb;
        plant_->CalcPointsPositions(*context_, frameB, p_BCb,
                                    plant_->world_frame(), &p_WCb);
        break;
      }
    }
    contact_wrench_sol.emplace_back(bodyA_index, bodyB_index,
                                    math::autoDiffToValueMatrix(p_WCb),
                                    Vector6<double>(F_Cb_W));
  }
  return contact_wrench_sol;
}

void StaticEquilibriumProblem::UpdateComplementaryTolerance(double tol) {
  DRAKE_DEMAND(tol >= 0);
  for (const auto& binding :
       static_friction_cone_complementary_nonlinear_constraints_) {
    binding.evaluator()->UpdateComplementaryTolerance(tol);
  }
}

}  // namespace multibody
}  // namespace drake
