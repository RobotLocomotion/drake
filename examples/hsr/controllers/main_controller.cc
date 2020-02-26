#include "drake/examples/hsr/controllers/main_controller.h"

#include "drake/examples/hsr/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace examples {
namespace hsr {
namespace controller {

MainController::MainController(
    const drake::multibody::MultibodyPlant<double>& robot_plant,
    const drake::multibody::MultibodyPlant<double>& welded_robot_plant,
    const RobotParameters<double>& parameters)
    : parameters_(parameters) {
  const int num_positions = robot_plant.num_positions();
  const int num_velocities = robot_plant.num_velocities();
  const int num_actuators = robot_plant.num_actuators();
  const int state_size = num_positions + num_velocities;

  drake::systems::DiagramBuilder<double> builder;

  auto& inverse_dynamics_controller =
      *builder.AddSystem<InverseDynamicsController>(
          robot_plant, welded_robot_plant, parameters);

  // Create a desired state passthrough to connect to different controllers.
  // Currently, there is only one controller.
  {
    auto& desired_state_passthrough =
        *builder.template AddSystem<drake::systems::PassThrough<double>>(
            state_size);

    builder.Connect(desired_state_passthrough.get_output_port(),
                    inverse_dynamics_controller.get_desired_state_input_port());

    // Exposes the desired state port.
    input_port_index_desired_state_ =
        builder.ExportInput(desired_state_passthrough.get_input_port());
  }

  // Create an estimated state passthrough to connect to different controllers.
  {
    auto& estimated_state_passthrough =
        *builder.template AddSystem<drake::systems::PassThrough<double>>(
            state_size);

    builder.Connect(
        estimated_state_passthrough.get_output_port(),
        inverse_dynamics_controller.get_estimated_state_input_port());

    // Exposes the estimated state port.
    input_port_index_estimated_state_ =
        builder.ExportInput(estimated_state_passthrough.get_input_port());
  }

  // Map the generalized force output of the welded plant back to the original
  // floating base plant.
  Eigen::MatrixXd inverse_dynamics_selector =
      Eigen::MatrixXd::Zero(num_velocities, num_velocities);
  inverse_dynamics_selector.bottomRightCorner(num_actuators, num_actuators) =
      Eigen::MatrixXd::Identity(num_actuators, num_actuators);

  // Create systems for the selectors.
  drake::systems::MatrixGain<double>& inverse_dynamics_selector_system =
      *builder.template AddSystem<drake::systems::MatrixGain<double>>(
          inverse_dynamics_selector);

  // Expose the generalized force output port.
  builder.Connect(
      inverse_dynamics_controller.get_generalized_force_output_port(),
      inverse_dynamics_selector_system.get_input_port());
  output_port_index_generalized_force_ =
      builder.ExportOutput(inverse_dynamics_selector_system.get_output_port());

  // Create a constant actuation source since this port is required by the
  // multibody plant.
  drake::VectorX<double> constant_actuation_value =
      drake::VectorX<double>::Zero(num_actuators);
  auto& actuation_constant_source =
      *builder.template AddSystem<drake::systems::ConstantVectorSource<double>>(
          constant_actuation_value);

  // Expose the constant actuation output port.
  output_port_index_actuation_ =
      builder.ExportOutput(actuation_constant_source.get_output_port());

  builder.BuildInto(this);
}

}  // namespace controller
}  // namespace hsr
}  // namespace examples
}  // namespace drake
