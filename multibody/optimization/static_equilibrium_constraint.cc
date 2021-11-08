#include "drake/multibody/optimization/static_equilibrium_constraint.h"

#include <unordered_map>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace {
int GetLambdaSize(const std::map<SortedPair<geometry::GeometryId>,
                                 GeometryPairContactWrenchEvaluatorBinding>&
                      contact_pair_to_wrench_evaluator) {
  int num_lambda = 0;
  for (const auto& term : contact_pair_to_wrench_evaluator) {
    num_lambda += term.second.contact_wrench_evaluator->num_lambda();
  }
  return num_lambda;
}
}  // namespace
StaticEquilibriumConstraint::StaticEquilibriumConstraint(
    const MultibodyPlant<AutoDiffXd>* plant,
    systems::Context<AutoDiffXd>* context,
    const std::map<SortedPair<geometry::GeometryId>,
                   GeometryPairContactWrenchEvaluatorBinding>&
        contact_pair_to_wrench_evaluator)
    : solvers::Constraint(plant->num_velocities(),
                          plant->num_positions() + plant->num_actuated_dofs() +
                              GetLambdaSize(contact_pair_to_wrench_evaluator),
                          Eigen::VectorXd::Zero(plant->num_velocities()),
                          Eigen::VectorXd::Zero(plant->num_velocities())),
      plant_{plant},
      context_{context},
      contact_pair_to_wrench_evaluator_(contact_pair_to_wrench_evaluator),
      B_actuation_{plant_->MakeActuationMatrix()} {}

void StaticEquilibriumConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_autodiff(num_constraints());
  DoEval(x.cast<AutoDiffXd>(), &y_autodiff);
  *y = math::ExtractValue(y_autodiff);
}

void StaticEquilibriumConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  const auto& q = x.head(plant_->num_positions());
  const auto& u =
      x.segment(plant_->num_positions(), plant_->num_actuated_dofs());
  *y = B_actuation_ * u;
  // TODO(hongkai.dai): Use UpdateContextConfiguration when it supports
  // MultibodyPlant<AutoDiffXd> and Context<AutoDiffXd>
  if (!internal::AreAutoDiffVecXdEqual(q, plant_->GetPositions(*context_))) {
    plant_->SetPositions(context_, q);
  }
  *y += plant_->CalcGravityGeneralizedForces(*context_);
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
  const int lambda_start_index_in_x =
      plant_->num_positions() + plant_->num_actuated_dofs();
  for (const auto& [contact_pair, evaluator] :
       contact_pair_to_wrench_evaluator_) {
    const geometry::SignedDistancePair<AutoDiffXd> signed_distance_pair =
        query_object.ComputeSignedDistancePairClosestPoints(
            contact_pair.first(), contact_pair.second());
    const geometry::FrameId frame_A_id =
        inspector.GetFrameId(signed_distance_pair.id_A);
    const geometry::FrameId frame_B_id =
        inspector.GetFrameId(signed_distance_pair.id_B);
    const Frame<AutoDiffXd>& frameA =
        plant_->GetBodyFromFrameId(frame_A_id)->body_frame();
    const Frame<AutoDiffXd>& frameB =
        plant_->GetBodyFromFrameId(frame_B_id)->body_frame();

    // Compute the Jacobian.
    // Define Body A's frame as A, the geometry attached to body A has frame Ga,
    // and the witness point on geometry Ga is Ca.
    const auto& X_AGa = inspector.GetPoseInFrame(signed_distance_pair.id_A);
    const auto& p_GaCa = signed_distance_pair.p_ACa;
    const Vector3<AutoDiffXd> p_ACa = X_AGa.cast<AutoDiffXd>() * p_GaCa;
    // Define Body B's frame as B, the geometry attached to body B has frame Gb,
    // and the witness point on geometry Gb is Cb.
    const auto& X_BGb = inspector.GetPoseInFrame(signed_distance_pair.id_B);
    const auto& p_GbCb = signed_distance_pair.p_BCb;
    const Vector3<AutoDiffXd> p_BCb = X_BGb.cast<AutoDiffXd>() * p_GbCb;
    Eigen::Matrix<AutoDiffXd, 6, Eigen::Dynamic> Jv_V_WCa(
        6, plant_->num_velocities());
    Eigen::Matrix<AutoDiffXd, 6, Eigen::Dynamic> Jv_V_WCb(
        6, plant_->num_velocities());
    plant_->CalcJacobianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                        frameA, p_ACa, plant_->world_frame(),
                                        plant_->world_frame(), &Jv_V_WCa);
    plant_->CalcJacobianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                        frameB, p_BCb, plant_->world_frame(),
                                        plant_->world_frame(), &Jv_V_WCb);

    // Find the lambda corresponding to the geometry pair (id_A, id_B).
    VectorX<AutoDiffXd> lambda(
        evaluator.contact_wrench_evaluator->num_lambda());

    for (int i = 0; i < lambda.rows(); ++i) {
      lambda(i) = x(lambda_start_index_in_x +
                    evaluator.lambda_indices_in_all_lambda[i]);
    }

    AutoDiffVecXd F_AB_W;
    evaluator.contact_wrench_evaluator->Eval(
        evaluator.contact_wrench_evaluator->ComposeVariableValues(*context_,
                                                                  lambda),
        &F_AB_W);

    // By definition, F_AB_W is the contact wrench applied to id_B from id_A,
    // at the contact point. By Newton's third law, the contact wrench applied
    // to id_A from id_B at the contact point is -F_AB_W.
    *y += Jv_V_WCa.transpose() * -F_AB_W + Jv_V_WCb.transpose() * F_AB_W;
  }
}
void StaticEquilibriumConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "StaticEquilibriumConstraint: does not support Eval with symbolic "
      "variable and expressions.");
}

solvers::Binding<StaticEquilibriumConstraint>
StaticEquilibriumConstraint::MakeBinding(
    const MultibodyPlant<AutoDiffXd>* plant,
    systems::Context<AutoDiffXd>* context,
    const std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                                VectorX<symbolic::Variable>>>&
        contact_wrench_evaluators_and_lambda,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& u_vars) {
  // contact_pair_to_wrench_evaluator will be used in the constructor of
  // StaticEquilibriumConstraint.
  std::map<SortedPair<geometry::GeometryId>,
           GeometryPairContactWrenchEvaluatorBinding>
      contact_pair_to_wrench_evaluator;
  // Get the total size of lambda used in this StaticEquilibriumConstraint. We
  // find the unique lambda variable for all contact wrench evaluators.
  // We will aggregate the lambda variables for each contact wrench evaluator
  // into a vector `all_lambda`. map_lambda_id_to_index records the index of
  // a lambda variable in the vector all_lambda.
  std::unordered_map<symbolic::Variable::Id, int> map_lambda_id_to_index;
  int lambda_count = 0;
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda) {
    const auto& contact_wrench_evaluator =
        contact_wrench_evaluator_and_lambda.first;
    const auto& lambda_i = contact_wrench_evaluator_and_lambda.second;
    DRAKE_DEMAND(contact_wrench_evaluator->num_lambda() == lambda_i.rows());
    std::vector<int> lambda_indices_in_all_lambda(lambda_i.rows());
    // Loop through each lambda variable bound with the contact wrench
    // evaluator, record the index of the lambda variable in all_lambda.
    for (int i = 0; i < contact_wrench_evaluator_and_lambda.second.rows();
         ++i) {
      const auto& id = contact_wrench_evaluator_and_lambda.second(i).get_id();
      auto it = map_lambda_id_to_index.find(id);
      if (it == map_lambda_id_to_index.end()) {
        lambda_indices_in_all_lambda[i] = lambda_count;
        map_lambda_id_to_index.emplace_hint(it, id, lambda_count++);
      } else {
        lambda_indices_in_all_lambda[i] = it->second;
      }
    }
    contact_pair_to_wrench_evaluator.emplace(
        contact_wrench_evaluator->geometry_id_pair(),
        GeometryPairContactWrenchEvaluatorBinding{lambda_indices_in_all_lambda,
                                                  contact_wrench_evaluator});
  }
  // Now compose the vector all_lambda.
  const int num_lambda = lambda_count;
  VectorX<symbolic::Variable> all_lambda(num_lambda);
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda) {
    const auto& lambda_i = contact_wrench_evaluator_and_lambda.second;
    for (int j = 0; j < lambda_i.rows(); ++j) {
      all_lambda(map_lambda_id_to_index.at(lambda_i[j].get_id())) = lambda_i(j);
    }
  }
  DRAKE_DEMAND(q_vars.rows() == plant->num_positions());
  DRAKE_DEMAND(u_vars.rows() == plant->num_actuated_dofs());
  // The bound variable for this StaticEquilibriumConstraint is q_u_lambda.
  VectorX<symbolic::Variable> q_u_lambda(
      plant->num_positions() + plant->num_actuated_dofs() + num_lambda);
  q_u_lambda << q_vars, u_vars, all_lambda;
  auto static_equilibrium_constraint =
      // Do not call make_shared because the constructor
      // StaticEquilibriumConstraint is private.
      std::shared_ptr<StaticEquilibriumConstraint>(
          new StaticEquilibriumConstraint(plant, context,
                                          contact_pair_to_wrench_evaluator));
  return solvers::Binding<StaticEquilibriumConstraint>(
      static_equilibrium_constraint, q_u_lambda);
}
}  // namespace multibody
}  // namespace drake
