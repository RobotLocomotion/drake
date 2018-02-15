#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"

#include <vector>

#include "drake/manipulation/schunk_wsg/schunk_wsg_low_level_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

SchunkWsgController::SchunkWsgController() {
  systems::DiagramBuilder<double> builder;

  // Add low-level controller. This makes the gripper behave as though it had
  // only one degree of freedom.
  auto low_level_controller = builder.AddSystem<SchunkWsgLowLevelController>();

  // Add position controller. This controls the separation of the fingers.
  auto position_controller = builder.AddSystem<SchunkWsgPositionController>();

  // Export the inputs.
  estimated_state_input_port_ = builder.ExportInput(
      low_level_controller->get_estimated_joint_state_input_port());
  desired_state_input_port_ =
      builder.ExportInput(position_controller->get_input_port_desired_state());
  max_force_input_port_ =
      builder.ExportInput(position_controller->get_max_force_input_port());

  // Export the outputs.
  builder.ExportOutput(
      low_level_controller->get_comanded_joint_force_output_port());

  // Connect the subsystems.
  // position_controller -> low_level_controller
  builder.Connect(position_controller->get_output_port_control(),
                  low_level_controller->get_commanded_grip_force_input_port());

  // low_level_controller -> position_controller
  builder.Connect(low_level_controller->get_estimated_grip_state_output_port(),
                  position_controller->get_input_port_estimated_state());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
