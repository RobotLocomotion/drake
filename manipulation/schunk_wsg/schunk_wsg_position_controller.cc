#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

#include <vector>

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
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

SchunkWsgPositionController::SchunkWsgPositionController() {
  systems::DiagramBuilder<double> builder;

  // Add the PID controller.
  // The p gain here is somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  const Vector1d wsg_kp{2000.0};
  const Vector1d wsg_ki{0.0};
  const Vector1d wsg_kd{5.0};

  auto pid_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          wsg_kp, wsg_ki, wsg_kd);

  estimated_state_input_port_ =
      builder.ExportInput(pid_controller->get_input_port_estimated_state());
  desired_state_input_port_ =
      builder.ExportInput(pid_controller->get_input_port_desired_state());

  // Add the saturation block.
  // Create a gain block to negate the max force (to produce a minimum
  // force).
  auto max_force_passthrough =
      builder.AddSystem<systems::PassThrough<double>>(1);
  max_force_input_port_ =
      builder.ExportInput(max_force_passthrough->get_input_port());
  auto positive_gain = builder.AddSystem<systems::MatrixGain<double>>(
      MatrixX<double>::Ones(1, 1));
  auto negative_gain = builder.AddSystem<systems::MatrixGain<double>>(
      -MatrixX<double>::Ones(1, 1));
  builder.Connect(max_force_passthrough->get_output_port(),
                  positive_gain->get_input_port());
  builder.Connect(max_force_passthrough->get_output_port(),
                  negative_gain->get_input_port());

  auto saturation = builder.AddSystem<systems::Saturation<double>>(1);

  builder.Connect(pid_controller->get_output_port_control(),
                  saturation->get_input_port());
  builder.Connect(positive_gain->get_output_port(),
                  saturation->get_max_value_port());
  builder.Connect(negative_gain->get_output_port(),
                  saturation->get_min_value_port());

  // Output the saturated force.
  builder.ExportOutput( saturation->get_output_port());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
