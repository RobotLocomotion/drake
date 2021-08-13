#include "drake/multibody/optimization/manipulator_equation_constraint.h"

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

ManipulatorEquationConstraint::ManipulatorEquationConstraint(
    const MultibodyPlant<AutoDiffXd>* plant,
    systems::Context<AutoDiffXd>* context,
    const std::map<SortedPair<geometry::GeometryId>,
                   GeometryPairContactWrenchEvaluatorBinding>&
        contact_pair_to_wrench_evaluator)
    : solvers::Constraint(
          plant->num_velocities(),
          plant->num_velocities() + plant->num_positions() +
              plant->num_velocities() + plant->num_actuated_dofs() +
              GetLambdaSize(contact_pair_to_wrench_evaluator) + 1 /* for dt */,
          Eigen::VectorXd::Zero(plant->num_velocities()),
          Eigen::VectorXd::Zero(plant->num_velocities())),
      plant_{plant},
      context_{context},
      contact_pair_to_wrench_evaluator_(contact_pair_to_wrench_evaluator),
      B_actuation_{plant_->MakeActuationMatrix()} {}

void ManipulatorEquationConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_autodiff(num_constraints());
  DoEval(x.cast<AutoDiffXd>(), &y_autodiff);
  *y = math::ExtractValue(y_autodiff);
}

// The format of the input to the Eval() function is a vector containing:
// {vₙ, qₙ₊₁, vₙ₊₁, uₙ₊₁, λₙ₊₁, dt},
// where λₙ₊₁ is a concatenation of lambdas for all contacts.
// TODO(rcory) Combine duplicate code between ManipulatorEquationConstraint and
//  StaticEquilibriumConstraint.
void ManipulatorEquationConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  const auto num_positions = plant_->num_positions();
  const auto num_velocities = plant_->num_velocities();
  const auto& v = x.head(num_velocities);
  const auto& qv_next =
      x.segment(num_velocities, num_positions + num_velocities);
  const auto& v_next =
      x.segment(num_velocities + num_positions, num_velocities);
  const auto& u_next =
      x.segment(num_velocities + num_positions + num_velocities,
                plant_->num_actuated_dofs());
  const auto& time_step = x.tail<1>();

  *y = B_actuation_ * u_next;

  UpdateContextPositionsAndVelocities(context_, *plant_, qv_next);
  *y += plant_->CalcGravityGeneralizedForces(*context_);  // g(q[n+1])

  // Calc the bias term C(qₙ₊₁, vₙ₊₁)
  Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1>
      C_bias(plant_->num_velocities(), 1);
  plant_->CalcBiasTerm(*context_, &C_bias);
  *y -= C_bias;

  const auto& query_port = plant_->get_geometry_query_input_port();
  if (!query_port.HasValue(*context_)) {
    throw std::invalid_argument(
        "ManipulatorEquationConstraint: Cannot get a valid "
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
  const int lambda_start_index_in_x = num_velocities + num_positions +
                                      num_velocities +
                                      plant_->num_actuated_dofs();
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    const geometry::FrameId frame_A_id =
        inspector.GetFrameId(signed_distance_pair.id_A);
    const geometry::FrameId frame_B_id =
        inspector.GetFrameId(signed_distance_pair.id_B);
    const Frame<AutoDiffXd>& frameA =
        plant_->GetBodyFromFrameId(frame_A_id)->body_frame();
    const Frame<AutoDiffXd>& frameB =
        plant_->GetBodyFromFrameId(frame_B_id)->body_frame();

    // Compute the Jacobian.
    // Define Body A's frame as A, the geometry attached to body A as frame Ga,
    // and the witness point on geometry Ga as Ca.
    const auto& X_AGa = inspector.GetPoseInFrame(signed_distance_pair.id_A);
    const auto& p_GaCa = signed_distance_pair.p_ACa;
    const Vector3<AutoDiffXd> p_ACa = X_AGa.cast<AutoDiffXd>() * p_GaCa;
    // Define Body B's frame as B, the geometry attached to body B as frame Gb,
    // and the witness point on geometry Gb as Cb.
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

    const SortedPair<geometry::GeometryId> contact_pair(
        signed_distance_pair.id_A, signed_distance_pair.id_B);
    // Find the lambda corresponding to the geometry pair (id_A, id_B).
    const auto it = contact_pair_to_wrench_evaluator_.find(contact_pair);
    if (it == contact_pair_to_wrench_evaluator_.end()) {
      throw std::runtime_error(
          "The input argument contact_pair_to_wrench_evaluator in the "
          "ManipulatorEquationConstraint constructor doesn't include all "
          "possible contact pairs.");
    }

    VectorX<AutoDiffXd> lambda(
        it->second.contact_wrench_evaluator->num_lambda());

    for (int i = 0; i < lambda.rows(); ++i) {
      lambda(i) = x(lambda_start_index_in_x +
                    it->second.lambda_indices_in_all_lambda[i]);
    }

    AutoDiffVecXd F_AB_W;
    it->second.contact_wrench_evaluator->Eval(
        it->second.contact_wrench_evaluator->ComposeVariableValues(*context_,
                                                                   lambda),
        &F_AB_W);

    // By definition, F_AB_W is the contact wrench applied to id_B from id_A,
    // at the contact point. By Newton's third law, the contact wrench applied
    // to id_A from id_B at the contact point is -F_AB_W.
    *y += Jv_V_WCa.transpose() * -F_AB_W + Jv_V_WCb.transpose() * F_AB_W;
  }

  Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, Eigen::Dynamic> M_mass(
      plant_->num_velocities(), plant_->num_velocities());
  plant_->CalcMassMatrixViaInverseDynamics(*context_, &M_mass);

  *y = *y * time_step - M_mass * (v_next - v);
}
void ManipulatorEquationConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "ManipulatorEquationConstraint: does not support Eval with symbolic "
      "variable and expressions.");
}

// This function binds a portion of the decision variables in the
// MathematicalProgram to a ManipulatorEquationConstraint, namely {vₙ, qₙ₊₁,
// vₙ₊₁, uₙ₊₁, λₙ₊₁}, where λ here represents the concatenation
// of all lambda. For contact implicit trajectory optimization, this binding
// should be made for each time sample in the trajectory.
// TODO(rcory) Combine duplicate code between ManipulatorEquationConstraint and
//  StaticEquilibriumConstraint.
solvers::Binding<ManipulatorEquationConstraint>
ManipulatorEquationConstraint::MakeBinding(
    const MultibodyPlant<AutoDiffXd>* plant,
    systems::Context<AutoDiffXd>* context,
    const std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                                VectorX<symbolic::Variable>>>&
        contact_wrench_evaluators_and_lambda,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& v_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_next_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& v_next_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& u_next_vars,
    const symbolic::Variable& dt_var) {

  // contact_pair_to_wrench_evaluator will be used in the constructor of
  // ManipulatorEquationConstraint.
  std::map<SortedPair<geometry::GeometryId>,
           GeometryPairContactWrenchEvaluatorBinding>
      contact_pair_to_wrench_evaluator;
  // Get the total size of lambda used in this ManipulatorEquationConstraint. We
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
  // Now compose the vector all_next_lambda.
  const int num_lambda = lambda_count;
  VectorX<symbolic::Variable> all_lambda_next(num_lambda);
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda) {
    const auto& lambda_i = contact_wrench_evaluator_and_lambda.second;
    for (int j = 0; j < lambda_i.rows(); ++j) {
      all_lambda_next(map_lambda_id_to_index.at(lambda_i[j].get_id())) =
          lambda_i(j);
    }
  }
  DRAKE_DEMAND(v_vars.rows() == plant->num_velocities());
  DRAKE_DEMAND(q_next_vars.rows() == plant->num_positions());
  DRAKE_DEMAND(v_next_vars.rows() == plant->num_velocities());
  DRAKE_DEMAND(u_next_vars.rows() == plant->num_actuated_dofs());

  // The bound variable for this ManipulatorEquationConstraint is
  // bound_x = {v, q_next, v_next, u_next, all_lambda_next, dt}.
  VectorX<symbolic::Variable> bound_x(
      plant->num_velocities() + plant->num_positions() +
      plant->num_velocities() + plant->num_actuated_dofs() + num_lambda + 1);
  bound_x << v_vars, q_next_vars, v_next_vars, u_next_vars, all_lambda_next,
      dt_var;
  auto manipulator_equation_constraint =
      // Do not call make_shared because the constructor
      // ManipulatorEquationConstraint is private.
      std::shared_ptr<ManipulatorEquationConstraint>(
          new ManipulatorEquationConstraint(plant, context,
                                            contact_pair_to_wrench_evaluator));
  return solvers::Binding<ManipulatorEquationConstraint>(
      manipulator_equation_constraint, bound_x);
}
}  // namespace multibody
}  // namespace drake
