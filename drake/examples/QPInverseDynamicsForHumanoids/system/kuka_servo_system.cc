#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_servo_system.h"

#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

KukaServoSystem::KukaServoSystem(const RigidBodyTree<double>& robot,
                                 const std::string& alias_groups_file_name,
                                 const std::string& param_file_name, double dt)
    : DiscreteTimePlanEvalSystem(robot, alias_groups_file_name, param_file_name,
                                 dt) {
  DRAKE_DEMAND(get_robot().get_num_velocities() ==
               get_robot().get_num_positions());
  input_port_index_desired_state_and_acceleration_ =
      DeclareInputPort(systems::kVectorValued,
                       get_robot().get_num_positions() +
                           2 * get_robot().get_num_velocities())
          .get_index();
  set_name("kuka_servo_plan_eval");
}

void KukaServoSystem::Initialize(systems::State<double>* state) {
  VectorSetpoint<double>& plan =
      get_mutable_plan<VectorSetpoint<double>>(state);
  plan = VectorSetpoint<double>(get_robot().get_num_velocities());
  get_paramset().LookupDesiredDofMotionGains(&(plan.mutable_Kp()),
                                             &(plan.mutable_Kd()));

  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input = get_paramset().MakeQpInput({}, /* contacts */
                                        {}, /* tracked bodies */
                                        get_alias_groups());
}

void KukaServoSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the plan from abstract state.
  VectorSetpoint<double>& plan =
      get_mutable_plan<VectorSetpoint<double>>(state);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_index_humanoid_status());

  // Gets the desired position and velocity.
  const systems::BasicVector<double>* desired = EvalVectorInput(
      context, input_port_index_desired_state_and_acceleration_);

  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input = get_paramset().MakeQpInput({}, /* contacts */
                                        {}, /* tracked bodies */
                                        get_alias_groups());

  for (int i = 0; i < get_robot().get_num_positions(); i++) {
    plan.mutable_desired_position()[i] = desired->GetAtIndex(i);
    plan.mutable_desired_velocity()[i] =
        desired->GetAtIndex(i + get_robot().get_num_positions());
    plan.mutable_desired_acceleration()[i] = desired->GetAtIndex(
        i + get_robot().get_num_positions() + get_robot().get_num_velocities());
  }

  // Update desired accelerations.
  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.ComputeTargetAcceleration(robot_status->position(),
                                     robot_status->velocity());
}

std::unique_ptr<systems::AbstractState> KukaServoSystem::AllocateAbstractState()
    const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  abstract_vals[get_abstract_state_index_plan()] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<VectorSetpoint<double>>(VectorSetpoint<double>()));
  abstract_vals[get_abstract_state_index_qp_input()] =
      std::unique_ptr<systems::AbstractValue>(new systems::Value<QpInput>(
          get_paramset().MakeQpInput({}, /* contacts */
                                     {}, /* tracked bodies */
                                     get_alias_groups())));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
