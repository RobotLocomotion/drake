#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"

#include <vector>

#include "drake/common/default_scalars.h"
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

template <typename T>
SchunkWsgController<T>::SchunkWsgController() {
  systems::DiagramBuilder<T> builder;

  // Add low-level controller. This makes the gripper behave as though it had
  // only one degree of freedom.
  auto low_level_controller =
      builder.template AddSystem<SchunkWsgLowLevelController<T>>();

  // Add position controller. This controls the separation of the fingers.
  auto position_controller =
      builder.template AddSystem<SchunkWsgPositionController<T>>();

  // Export the inputs.
  estimated_state_input_port_ = builder.ExportInput(
      low_level_controller->get_input_port_estimated_joint_state());
  desired_state_input_port_ =
      builder.ExportInput(position_controller->get_input_port_desired_state());
  max_force_input_port_ =
      builder.ExportInput(position_controller->get_input_port_max_force());

  // Export the outputs.
  builder.ExportOutput(
      low_level_controller->get_output_port_commanded_joint_force());

  // Connect the subsystems.
  // position_controller -> low_level_controller
  builder.Connect(position_controller->get_output_port_control(),
                  low_level_controller->get_input_port_commanded_grip_force());

  // low_level_controller -> position_controller
  builder.Connect(low_level_controller->get_output_port_estimated_grip_state(),
                  position_controller->get_input_port_estimated_state());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::manipulation::schunk_wsg::SchunkWsgController);
