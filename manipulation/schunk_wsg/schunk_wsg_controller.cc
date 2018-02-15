#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

SchunkWsgController::SchunkWsgController() {
  systems::DiagramBuilder<double> builder;

  auto state_pass_through = builder.AddSystem<systems::PassThrough<double>>(
      kSchunkWsgNumPositions + kSchunkWsgNumVelocities);

  auto wsg_trajectory_generator =
      builder.AddSystem<SchunkWsgTrajectoryGenerator>(
          kSchunkWsgNumPositions + kSchunkWsgNumVelocities,
          kSchunkWsgPositionIndex);

  // Add the LCM-free controller.
  auto wsg_controller = builder.AddSystem<SchunkWsgPlainController>();

  // Export the inputs.
  command_input_port_ =
      builder.ExportInput(wsg_trajectory_generator->get_command_input_port());
  state_input_port_ = builder.ExportInput(state_pass_through->get_input_port());

  // Export the outputs.
  builder.ExportOutput(wsg_controller->get_output_port_control());

  // Connect the subsystems.
  // state_state_pass_through -> other subsystems
  builder.Connect(state_pass_through->get_output_port(),
                  wsg_trajectory_generator->get_state_input_port());
  builder.Connect(state_pass_through->get_output_port(),
                  wsg_controller->get_input_port_estimated_state());

  // wsg_trajectory_generator -> position_controller
  builder.Connect(wsg_trajectory_generator->get_target_output_port(),
                  wsg_controller->get_input_port_desired_state());
  builder.Connect(wsg_trajectory_generator->get_max_force_output_port(),
                  wsg_controller->get_max_force_input_port());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
