#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

#include <vector>

#include "drake/common/default_scalars.h"
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
SchunkWsgPositionController<T>::SchunkWsgPositionController() {
  systems::DiagramBuilder<T> builder;

  // Add the PID controller.
  // The p gain here is somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  const Vector1d kp{2000.0};
  const Vector1d ki{0.0};
  const Vector1d kd{5.0};

  const auto pid_controller =
      builder.template AddSystem<systems::controllers::PidController<T>>(kp, ki,
                                                                         kd);

  // Add a passthrough to distribute the max force to multiple subsystems.
  const auto max_force_passthrough =
      builder.template AddSystem<systems::PassThrough<T>>(1);

  // Add a gain block to negate the max force (to produce a minimum
  // force).
  const auto min_force = builder.template AddSystem<systems::MatrixGain<T>>(
      Vector1d{-1.0});

  // Add the saturation block.
  const auto saturation = builder.template AddSystem<systems::Saturation<T>>(1);

  // Export the inputs.
  estimated_state_input_port_ =
      builder.ExportInput(pid_controller->get_input_port_estimated_state());
  desired_state_input_port_ =
      builder.ExportInput(pid_controller->get_input_port_desired_state());
  max_force_input_port_ =
      builder.ExportInput(max_force_passthrough->get_input_port());

  // Export the outputs.
  builder.ExportOutput(saturation->get_output_port());

  // Connect the subsystems.
  // Max force -> other subsystems
  builder.Connect(max_force_passthrough->get_output_port(),
                  min_force->get_input_port());
  builder.Connect(max_force_passthrough->get_output_port(),
                  saturation->get_max_value_port());
  // Min force -> saturation
  builder.Connect(min_force->get_output_port(),
                  saturation->get_min_value_port());
  // PID -> saturation
  builder.Connect(pid_controller->get_output_port_control(),
                  saturation->get_input_port());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::manipulation::schunk_wsg::SchunkWsgPositionController);
