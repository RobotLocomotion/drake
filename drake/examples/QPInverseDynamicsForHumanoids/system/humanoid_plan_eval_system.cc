#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_plan_eval_system.h"

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
struct SimpleValkyriePlan {
  VectorSetpoint<double> joint_PDff;
  CartesianSetpoint<double> pelvis_PDff;
  CartesianSetpoint<double> torso_PDff;
  VectorSetpoint<double> com_PDff;

  Vector3<double> initial_com;
};

HumanoidPlanEvalSystem::HumanoidPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : DiscreteTimePlanEvalSystem(robot, alias_groups_file_name, param_file_name,
                                 dt) {
  set_name("humanoid_plan_eval");
}

void HumanoidPlanEvalSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the plan from abstract state.
  SimpleValkyriePlan& plan = get_mutable_plan<SimpleValkyriePlan>(state);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_index_humanoid_status());

  // Mutates the plan.
  // This can be much more complicated like replanning trajectories, etc.
  // Moves desired com height in a sine wave.
  plan.com_PDff.mutable_desired_position()[2] =
      plan.initial_com[2] + 0.1 * std::sin(robot_status->time() * 2 * M_PI);

  // Generates a QpInput and stores it in AbstractState.
  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input =
      get_paramset().MakeQpInput({"feet"},            /* contacts */
                                 {"pelvis", "torso"}, /* tracked bodies */
                                 get_alias_groups());

  // Does acceleration feedback based on the plan.
  qp_input.mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .tail<3>() = get_robot().getMass() *
                   plan.com_PDff.ComputeTargetAcceleration(
                       robot_status->com(), robot_status->comd());

  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.joint_PDff.ComputeTargetAcceleration(robot_status->position(),
                                                robot_status->velocity());

  const std::string& pelvis_body_name =
      get_alias_groups().get_body("pelvis")->get_name();
  const std::string& torso_body_name =
      get_alias_groups().get_body("torso")->get_name();

  qp_input.mutable_desired_body_motions()
      .at(pelvis_body_name)
      .mutable_values() = plan.pelvis_PDff.ComputeTargetAcceleration(
      robot_status->pelvis().pose(), robot_status->pelvis().velocity());

  qp_input.mutable_desired_body_motions().at(torso_body_name).mutable_values() =
      plan.torso_PDff.ComputeTargetAcceleration(
          robot_status->torso().pose(), robot_status->torso().velocity());
}

std::unique_ptr<systems::AbstractState>
HumanoidPlanEvalSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  abstract_vals[get_abstract_state_index_plan()] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<SimpleValkyriePlan>(SimpleValkyriePlan()));
  abstract_vals[get_abstract_state_index_qp_input()] =
      std::unique_ptr<systems::AbstractValue>(new systems::Value<QpInput>(
          get_paramset().MakeQpInput({"feet"},            /* contacts */
                                     {"pelvis", "torso"}, /* tracked bodies */
                                     get_alias_groups())));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

void HumanoidPlanEvalSystem::SetDesired(const VectorX<double>& q_d,
                                        systems::State<double>* state) {
  // Get the plan.
  SimpleValkyriePlan& plan = get_mutable_plan<SimpleValkyriePlan>(state);

  KinematicsCache<double> cache = get_robot().doKinematics(q_d);

  plan.initial_com = get_robot().centerOfMass(cache);
  plan.pelvis_PDff.mutable_desired_pose() = get_robot().relativeTransform(
      cache, 0, get_alias_groups().get_body("pelvis")->get_body_index());
  plan.torso_PDff.mutable_desired_pose() = get_robot().relativeTransform(
      cache, 0, get_alias_groups().get_body("torso")->get_body_index());

  plan.com_PDff = VectorSetpoint<double>(3);
  plan.com_PDff.mutable_desired_position() = plan.initial_com;

  int dim = get_robot().get_num_velocities();
  plan.joint_PDff = VectorSetpoint<double>(dim);
  plan.joint_PDff.mutable_desired_position() = q_d;

  // Sets the gains
  get_paramset().LookupDesiredBodyMotionGains(
      *get_alias_groups().get_body("pelvis"), &(plan.pelvis_PDff.mutable_Kp()),
      &(plan.pelvis_PDff.mutable_Kd()));
  get_paramset().LookupDesiredBodyMotionGains(
      *get_alias_groups().get_body("torso"), &(plan.torso_PDff.mutable_Kp()),
      &(plan.torso_PDff.mutable_Kd()));
  get_paramset().LookupDesiredDofMotionGains(&(plan.joint_PDff.mutable_Kp()),
                                             &(plan.joint_PDff.mutable_Kd()));
  Vector6<double> Kp, Kd;
  get_paramset().LookupDesiredCentroidalMomentumDotGains(&Kp, &Kd);
  plan.com_PDff.mutable_Kp() = Kp.tail<3>();
  plan.com_PDff.mutable_Kd() = Kd.tail<3>();
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
