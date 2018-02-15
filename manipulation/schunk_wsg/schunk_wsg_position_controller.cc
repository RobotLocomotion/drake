#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

#include <vector>

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_force_controller.h"
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

  auto estimated_state_passthrough =
      builder.AddSystem<systems::PassThrough<double>>(kSchunkWsgNumPositions +
                                                      kSchunkWsgNumVelocities);
  estimated_state_input_port_ =
      builder.ExportInput(estimated_state_passthrough->get_input_port());

  // Add the PID controller.
  // Set up the pre-saturation control, u, signal.
  //  q0 = (qR - qL)
  //  q0d = command
  MatrixX<double> J_q{1, 2};
  J_q << 0.5, -0.5;
  MatrixX<double> J_x{2, 4};
  J_x << J_q, MatrixX<double>::Zero(1, 2), MatrixX<double>::Zero(1, 2), J_q;

  auto convert_to_x_tilde = builder.AddSystem<systems::MatrixGain<double>>(J_x);
  builder.Connect(estimated_state_passthrough->get_output_port(),
                  convert_to_x_tilde->get_input_port());

  // The p gain here is somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  const Eigen::VectorXd wsg_kp = Eigen::VectorXd::Constant(J_q.rows(), 2000.0);
  const Eigen::VectorXd wsg_ki = Eigen::VectorXd::Constant(J_q.rows(), 0.0);
  const Eigen::VectorXd wsg_kd = Eigen::VectorXd::Constant(J_q.rows(), 5.0);

  auto wsg_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          wsg_kp, wsg_ki, wsg_kd);

  builder.Connect(convert_to_x_tilde->get_output_port(),
                  wsg_controller->get_input_port_estimated_state());
  desired_state_input_port_ =
      builder.ExportInput(wsg_controller->get_input_port_desired_state());

  // The PID controller outputs u_tilde, which corresponds to the feed-forward
  // force.

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

  builder.Connect(wsg_controller->get_output_port_control(),
                  saturation->get_input_port());
  builder.Connect(positive_gain->get_output_port(),
                  saturation->get_max_value_port());
  builder.Connect(negative_gain->get_output_port(),
                  saturation->get_min_value_port());

  // Add the force controller.
  auto force_controller = builder.AddSystem<SchunkWsgForceController>();
  builder.Connect(estimated_state_passthrough->get_output_port(),
                  force_controller->get_state_input_port());
  builder.Connect(saturation->get_output_port(),
                  force_controller->get_feed_forward_force_input_port());
  builder.ExportOutput(force_controller->get_output_port(0));
  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
