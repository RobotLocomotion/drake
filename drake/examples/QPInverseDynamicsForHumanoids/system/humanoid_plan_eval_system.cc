#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_plan_eval_system.h"

#include <memory>
#include <string>
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
struct SimpleStandingPlan {
  VectorSetpoint<double> joint_servo;
  CartesianSetpoint<double> pelvis_servo;
  CartesianSetpoint<double> torso_servo;
  VectorSetpoint<double> com_servo;

  Vector3<double> initial_com;
};

HumanoidPlanEvalSystem::HumanoidPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt),
      abs_state_index_plan_(0) {
  set_name("HumanoidPlanEval");
}

void HumanoidPlanEvalSystem::DoExtendedCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {}

std::unique_ptr<systems::AbstractValue>
HumanoidPlanEvalSystem::ExtendedAllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_ABORT_MSG(
      "HumanoidPlanEvalSystem does not have additional abstract output ports.");
}

void HumanoidPlanEvalSystem::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the plan from abstract state.
  SimpleStandingPlan& plan = get_mutable_abstract_value<SimpleStandingPlan>(
      state, abs_state_index_plan_);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_humanoid_status().get_index());

  // Mutates the plan given the current state.
  // This can be much more complicated like replanning trajectories, etc.
  // Moves desired com height in a sine wave.
  plan.com_servo.mutable_desired_position()[2] =
      plan.initial_com[2] + 0.1 * std::sin(context.get_time() * 2 * M_PI);

  // Generates a QpInput and stores it in AbstractState.
  QpInput& qp_input = get_mutable_qp_input(state);

  // Does acceleration feedback based on the plan.
  qp_input.mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .tail<3>() = get_robot().getMass() *
                   plan.com_servo.ComputeTargetAcceleration(
                       robot_status->com(), robot_status->comd());

  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.joint_servo.ComputeTargetAcceleration(robot_status->position(),
                                                 robot_status->velocity());

  const std::string& pelvis_body_name =
      get_alias_groups().get_body("pelvis")->get_name();
  const std::string& torso_body_name =
      get_alias_groups().get_body("torso")->get_name();

  qp_input.mutable_desired_body_motions()
      .at(pelvis_body_name)
      .mutable_values() = plan.pelvis_servo.ComputeTargetAcceleration(
      robot_status->pelvis().pose(), robot_status->pelvis().velocity());

  qp_input.mutable_desired_body_motions().at(torso_body_name).mutable_values() =
      plan.torso_servo.ComputeTargetAcceleration(
          robot_status->torso().pose(), robot_status->torso().velocity());
}

std::vector<std::unique_ptr<systems::AbstractValue>>
HumanoidPlanEvalSystem::ExtendedAllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(
      get_num_extended_abstract_states());
  abstract_vals[abs_state_index_plan_] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<SimpleStandingPlan>(SimpleStandingPlan()));
  return abstract_vals;
}

void HumanoidPlanEvalSystem::Initialize(const VectorX<double>& q_d,
                                        systems::State<double>* state) {
  // Gets the plan.
  SimpleStandingPlan& plan = get_mutable_abstract_value<SimpleStandingPlan>(
      state, abs_state_index_plan_);

  KinematicsCache<double> cache = get_robot().doKinematics(q_d);

  plan.initial_com = get_robot().centerOfMass(cache);
  plan.pelvis_servo.mutable_desired_pose() = get_robot().relativeTransform(
      cache, 0, get_alias_groups().get_body("pelvis")->get_body_index());
  plan.torso_servo.mutable_desired_pose() = get_robot().relativeTransform(
      cache, 0, get_alias_groups().get_body("torso")->get_body_index());

  plan.com_servo = VectorSetpoint<double>(3);
  plan.com_servo.mutable_desired_position() = plan.initial_com;

  int dim = get_robot().get_num_velocities();
  plan.joint_servo = VectorSetpoint<double>(dim);
  plan.joint_servo.mutable_desired_position() = q_d;

  // Sets the gains.
  get_paramset().LookupDesiredBodyMotionGains(
      *get_alias_groups().get_body("pelvis"), &(plan.pelvis_servo.mutable_Kp()),
      &(plan.pelvis_servo.mutable_Kd()));
  get_paramset().LookupDesiredBodyMotionGains(
      *get_alias_groups().get_body("torso"), &(plan.torso_servo.mutable_Kp()),
      &(plan.torso_servo.mutable_Kd()));
  get_paramset().LookupDesiredDofMotionGains(&(plan.joint_servo.mutable_Kp()),
                                             &(plan.joint_servo.mutable_Kd()));
  Vector6<double> Kp, Kd;
  get_paramset().LookupDesiredCentroidalMomentumDotGains(&Kp, &Kd);
  plan.com_servo.mutable_Kp() = Kp.tail<3>();
  plan.com_servo.mutable_Kd() = Kd.tail<3>();

  // Initializes qp input.
  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input =
      get_paramset().MakeQpInput({"feet"},            /* contacts */
                                 {"pelvis", "torso"}, /* tracked bodies */
                                 get_alias_groups());
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
