#include "drake/manipulation/schunk_wsg/schunk_wsg_low_level_controller.h"

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

SchunkWsgLowLevelController::SchunkWsgLowLevelController() {
  systems::DiagramBuilder<double> builder;

  auto estimated_state_passthrough =
      builder.AddSystem<systems::PassThrough<double>>(kSchunkWsgNumPositions +
                                                      kSchunkWsgNumVelocities);
  estimated_joint_state_input_port_ =
      builder.ExportInput(estimated_state_passthrough->get_input_port());

  // The mean finger position should be zero.
  auto desired_mean_finger_state =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Vector2<double>::Zero());

  // Add the PID controller.
  // Set up the control signal, u.
  MatrixX<double> mean_finger_position_jacobian{1, 2};
  mean_finger_position_jacobian << 0.5, 0.5;
  MatrixX<double> mean_finger_state_jacobian{2, 4};
  mean_finger_state_jacobian << mean_finger_position_jacobian,
      MatrixX<double>::Zero(1, 2), MatrixX<double>::Zero(1, 2),
      mean_finger_position_jacobian;

  auto convert_to_x_tilde = builder.AddSystem<systems::MatrixGain<double>>(
      mean_finger_state_jacobian);
  builder.Connect(estimated_state_passthrough->get_output_port(),
                  convert_to_x_tilde->get_input_port());

  const int num_pid_positions = mean_finger_position_jacobian.rows();
  const Eigen::VectorXd wsg_kp =
      Eigen::VectorXd::Constant(num_pid_positions, 2000.0);
  const Eigen::VectorXd wsg_ki =
      Eigen::VectorXd::Constant(num_pid_positions, 0.0);
  const Eigen::VectorXd wsg_kd =
      Eigen::VectorXd::Constant(num_pid_positions, 5.0);

  auto wsg_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          wsg_kp, wsg_ki, wsg_kd);

  builder.Connect(convert_to_x_tilde->get_output_port(),
                  wsg_controller->get_input_port_estimated_state());
  builder.Connect(desired_mean_finger_state->get_output_port(),
                  wsg_controller->get_input_port_desired_state());

  // The PID controller outputs u_tilde. Add a matrix gain block to convert
  // to u.
  auto convert_to_u = builder.AddSystem<systems::MatrixGain<double>>(
      mean_finger_position_jacobian.transpose());
  builder.Connect(wsg_controller->get_output_port_control(),
                  convert_to_u->get_input_port());
  auto adder = builder.AddSystem<systems::Adder<double>>(2, 2);
  builder.Connect(convert_to_u->get_output_port(), adder->get_input_port(0));
  auto convert_feed_forward_force_to_u =
      builder.AddSystem<systems::MatrixGain<double>>(
          (MatrixX<double>(2, 1) << 0.5, -0.4).finished());
  builder.Connect(convert_feed_forward_force_to_u->get_output_port(),
                  adder->get_input_port(1));
  commanded_grip_force_input_port_ =
      builder.ExportInput(convert_feed_forward_force_to_u->get_input_port());
  commanded_joint_force_output_port_ =
      builder.ExportOutput(adder->get_output_port());

  // Set up the estimated grip state output.
  MatrixX<double> grip_position_jacobian{1, 2};
  grip_position_jacobian << 0.5, -0.5;
  MatrixX<double> grip_state_jacobian{2, 4};
  grip_state_jacobian << grip_position_jacobian, MatrixX<double>::Zero(1, 2),
      MatrixX<double>::Zero(1, 2), grip_position_jacobian;

  auto convert_to_grip_state =
      builder.AddSystem<systems::MatrixGain<double>>(grip_state_jacobian);
  builder.Connect(estimated_state_passthrough->get_output_port(),
                  convert_to_grip_state->get_input_port());
  estimated_grip_state_output_port_ =
      builder.ExportOutput(convert_to_grip_state->get_output_port());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
