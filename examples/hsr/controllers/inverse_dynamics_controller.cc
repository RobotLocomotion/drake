#include "drake/examples/hsr/controllers/inverse_dynamics_controller.h"

#include "drake/examples/hsr/parameters/sim_parameters.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace hsr {
namespace controllers {

InverseDynamicsController::InverseDynamicsController(
    const multibody::MultibodyPlant<double>& robot_plant,
    const multibody::MultibodyPlant<double>& welded_robot_plant,
    const hsr::parameters::RobotParameters<double>& parameters) {
  // Confirm that the number of actuators should be same between the floating
  // base model and fixed base model
  DRAKE_DEMAND(robot_plant.num_actuators() ==
               welded_robot_plant.num_actuators());
  // Confirm the welded version of the plant is fully actuated.
  DRAKE_DEMAND(welded_robot_plant.num_actuators() ==
               welded_robot_plant.num_velocities());

  // Set up PID gains for the inverse dynamics controller.
  const int num_positions = robot_plant.num_positions();
  const int num_velocities = robot_plant.num_velocities();
  const int num_actuators = robot_plant.num_actuators();
  const int state_size = num_positions + num_velocities;

  const auto& sim_params = hsr::parameters::hsr_sim_flags();
  VectorX<double> Kp = sim_params.kp * VectorX<double>::Ones(num_actuators);
  VectorX<double> Ki = sim_params.ki * VectorX<double>::Ones(num_actuators);
  VectorX<double> Kd = sim_params.kd * VectorX<double>::Ones(num_actuators);
  LoadPidGainsFromRobotParameters(parameters, welded_robot_plant, &Kp, &Ki,
                                  &Kd);

  systems::DiagramBuilder<double> builder;
  // Create a default inverse dynamics controller.
  auto& inverse_dynamics_controller = *builder.template AddSystem<
      systems::controllers::InverseDynamicsController<double>>(
      welded_robot_plant, Kp, Ki, Kd, false);

  // Create a matrix to map the original floating base robot state to the
  // welded robot state.
  Eigen::MatrixXd feedback_selector =
      Eigen::MatrixXd::Zero(2 * num_actuators, state_size);
  for (multibody::JointActuatorIndex i(0); i < num_actuators; i++) {
    const auto& welded_joint_i =
        welded_robot_plant.get_joint_actuator(i).joint();
    const auto& float_joint_i = robot_plant.get_joint_actuator(i).joint();
    feedback_selector(welded_joint_i.position_start(),
                      float_joint_i.position_start()) = 1;
    feedback_selector(welded_joint_i.velocity_start() + num_actuators,
                      float_joint_i.velocity_start() + num_positions) = 1;
  }

  // Use Gain system to convert robot plant desired state to IDC state input.
  systems::MatrixGain<double>& select_desired_states =
      *builder.template AddSystem<systems::MatrixGain<double>>(
          feedback_selector);
  builder.Connect(select_desired_states.get_output_port(),
                  inverse_dynamics_controller.get_input_port_desired_state());

  // Use Gain system to convert robot plant estimated state to IDC state input.
  systems::MatrixGain<double>& select_estimated_states =
      *builder.template AddSystem<systems::MatrixGain<double>>(
          feedback_selector);
  builder.Connect(select_estimated_states.get_output_port(),
                  inverse_dynamics_controller.get_input_port_estimated_state());

  // Select generalized control signal of the welded plant and feed into
  // the original plant.
  Eigen::MatrixXd generalized_actuation_selector =
      Eigen::MatrixXd::Zero(num_velocities, num_actuators);
  generalized_actuation_selector.bottomRightCorner(num_actuators,
                                                   num_actuators) =
      Eigen::MatrixXd::Identity(num_actuators, num_actuators);

  systems::MatrixGain<double>& select_generalized_actuation_states =
      *builder.template AddSystem<systems::MatrixGain<double>>(
          generalized_actuation_selector);

  builder.Connect(inverse_dynamics_controller.get_output_port_control(),
                  select_generalized_actuation_states.get_input_port());

  // Exposes the estimated state port.
  input_port_index_estimated_state_ =
      builder.ExportInput(select_estimated_states.get_input_port());

  // Exposes the desired state port.
  input_port_index_desired_state_ =
      builder.ExportInput(select_desired_states.get_input_port());

  // Exposes the generalized force output port.
  output_port_index_generalized_force_ = builder.ExportOutput(
      select_generalized_actuation_states.get_output_port());

  builder.BuildInto(this);
}

void InverseDynamicsController::LoadPidGainsFromRobotParameters(
    const hsr::parameters::RobotParameters<double>& parameters,
    const multibody::MultibodyPlant<double>& welded_robot_plant,
    VectorX<double>* kp, VectorX<double>* kd, VectorX<double>* ki) {
  // Confirm the passed in plant is fully actuated.
  DRAKE_DEMAND(welded_robot_plant.num_actuators() ==
               welded_robot_plant.num_velocities());
  for (auto const& [part_name, part_parameters] : parameters.parts_parameters) {
    drake::log()->info(
        "Loading PID gains for Inverse Dynamics Controllers from: " +
        part_name);
    for (const auto& joint_parameters : part_parameters.joints_parameters) {
      const int joint_index =
          welded_robot_plant.GetJointByName(joint_parameters.name).index();
      (*kp)[joint_index] = joint_parameters.pid_gains.kp;
      (*kd)[joint_index] = joint_parameters.pid_gains.kd;
      (*ki)[joint_index] = joint_parameters.pid_gains.ki;
    }
  }
}

}  // namespace controllers
}  // namespace hsr
}  // namespace examples
}  // namespace drake
