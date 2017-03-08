#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// Example struct of a "Plan" object containing internal state such as desired
// trajectories or setpoints.
struct SimplePlan {
  VectorSetpoint<double> joint_PDff;
  CartesianSetpoint<double> pelvis_PDff;
  CartesianSetpoint<double> torso_PDff;
  VectorSetpoint<double> com_PDff;

  Vector3<double> initial_com;
};

PlanEvalSystem::PlanEvalSystem(const RigidBodyTree<double>& robot)
    : robot_(robot), alias_groups_(robot) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();
  // Declare discrete time controller.
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

  set_name("plan_eval");

  std::string alias_groups_config =
      drake::GetDrakePath() + "/examples/QPInverseDynamicsForHumanoids/"
      "config/valkyrie.alias_groups";
  std::string controller_config =
      drake::GetDrakePath() + "/examples/QPInverseDynamicsForHumanoids/"
      "config/controller.yaml";

  // KinematicsProperty
  alias_groups_.LoadFromFile(alias_groups_config);

  // Controller config
  paramset_.LoadFromYAMLConfigFile(YAML::LoadFile(controller_config),
                                   alias_groups_);
}

void PlanEvalSystem::DoCalcOutput(const systems::Context<double>& context,
                                  systems::SystemOutput<double>* output) const {
  // Output:
  QpInput& qp_input = output->GetMutableData(output_port_index_qp_input_)
                          ->GetMutableValue<QpInput>();

  // Gets QpInput from AbstractState.
  qp_input = context.get_abstract_state<QpInput>(1);
}

void PlanEvalSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  systems::AbstractState* abs_state = state->get_mutable_abstract_state();
  DRAKE_DEMAND(abs_state->size() == 2);

  // Gets the plan from abstract state.
  SimplePlan& plan =
      abs_state->get_mutable_abstract_state(0).GetMutableValue<SimplePlan>();

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  /////////////////////////////////////////////////////////////////////////////
  // Mutates the plan.
  // This can be much more complicated like replanning trajectories, etc.
  paramset_.LookupDesiredBodyMotionGains(*robot_.FindBody("pelvis"),
                                         &(plan.pelvis_PDff.mutable_Kp()),
                                         &(plan.pelvis_PDff.mutable_Kd()));
  paramset_.LookupDesiredBodyMotionGains(*robot_.FindBody("torso"),
                                         &(plan.torso_PDff.mutable_Kp()),
                                         &(plan.torso_PDff.mutable_Kd()));
  paramset_.LookupDesiredDofMotionGains(&(plan.joint_PDff.mutable_Kp()),
                                        &(plan.joint_PDff.mutable_Kd()));

  Vector6<double> Kp, Kd;
  paramset_.LookupDesiredCentroidalMomentumDotGains(&Kp, &Kd);
  plan.com_PDff.mutable_Kp() = Kp.tail<3>();
  plan.com_PDff.mutable_Kd() = Kd.tail<3>();

  // Moves desired com height in a sine wave.
  plan.com_PDff.mutable_desired_position()[2] =
      plan.initial_com[2] + 0.1 * std::sin(robot_status->time() * 2 * M_PI);

  /////////////////////////////////////////////////////////////////////////////
  // Generates a QpInput and store it in AbstractState.
  QpInput& qp_input =
      abs_state->get_mutable_abstract_state(1).GetMutableValue<QpInput>();
  qp_input = QpInput(GetDofNames(robot_));
  qp_input.mutable_contact_information() =
      paramset_.MakeContactInformation("feet", alias_groups_);

  std::unordered_map<std::string, DesiredBodyMotion> motion_d =
      paramset_.MakeDesiredBodyMotion("pelvis", alias_groups_);
  qp_input.mutable_desired_body_motions().insert(motion_d.begin(),
                                                 motion_d.end());
  motion_d = paramset_.MakeDesiredBodyMotion("torso", alias_groups_);
  qp_input.mutable_desired_body_motions().insert(motion_d.begin(),
                                                 motion_d.end());

  qp_input.mutable_desired_dof_motions() = paramset_.MakeDesiredDofMotions();
  qp_input.mutable_w_basis_reg() = paramset_.get_basis_regularization_weight();

  qp_input.mutable_desired_centroidal_momentum_dot() =
      paramset_.MakeDesiredCentroidalMomentumDot();

  // Does acceleration feedback based on the plan.
  qp_input.mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .tail<3>() = robot_.getMass() *
                   plan.com_PDff.ComputeTargetAcceleration(
                       robot_status->com(), robot_status->comd());

  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.joint_PDff.ComputeTargetAcceleration(robot_status->position(),
                                                robot_status->velocity());

  qp_input.mutable_desired_body_motions().at("pelvis").mutable_values() =
      plan.pelvis_PDff.ComputeTargetAcceleration(
          robot_status->pelvis().pose(), robot_status->pelvis().velocity());
  qp_input.mutable_desired_body_motions().at("torso").mutable_values() =
      plan.torso_PDff.ComputeTargetAcceleration(
          robot_status->torso().pose(), robot_status->torso().velocity());
}

std::unique_ptr<systems::AbstractState> PlanEvalSystem::AllocateAbstractState()
    const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.reserve(2);
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<SimplePlan>(SimplePlan())));
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<QpInput>(QpInput(GetDofNames(robot_)))));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

std::unique_ptr<systems::AbstractValue> PlanEvalSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == output_port_index_qp_input_);
  return systems::AbstractValue::Make<QpInput>(QpInput(GetDofNames(robot_)));
}

void PlanEvalSystem::SetDesired(const VectorX<double>& q_d,
                                systems::State<double>* state) {
  systems::AbstractState* abs_state = state->get_mutable_abstract_state();
  DRAKE_DEMAND(abs_state->size() == 2);

  // Get the plan.
  SimplePlan& plan =
      abs_state->get_mutable_abstract_state(0).GetMutableValue<SimplePlan>();

  KinematicsCache<double> cache = robot_.doKinematics(q_d);

  plan.initial_com = robot_.centerOfMass(cache);
  plan.pelvis_PDff.mutable_desired_pose() = robot_.relativeTransform(
      cache, 0,
      alias_groups_.get_body_group("pelvis").front()->get_body_index());
  plan.torso_PDff.mutable_desired_pose() = robot_.relativeTransform(
      cache, 0,
      alias_groups_.get_body_group("torso").front()->get_body_index());

  plan.com_PDff = VectorSetpoint<double>(3);
  plan.com_PDff.mutable_desired_position() = plan.initial_com;

  int dim = robot_.get_num_velocities();
  plan.joint_PDff = VectorSetpoint<double>(dim);
  plan.joint_PDff.mutable_desired_position() = q_d;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
