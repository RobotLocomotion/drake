#include "drake/manipulation/schunk_wsg/schunk_wsg_force_controller.h"

#include <vector>

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

SchunkWsgForceController::SchunkWsgForceController() {
  systems::DiagramBuilder<double> builder;

  // The mean finger position should be zero.
  auto desired_mean_finger_state =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Vector2<double>::Zero());

  // Add the PID controller.
  // Set up the control signal, u.
  MatrixX<double> J_q{1, 2};
  J_q << 0.5, 0.5;
  MatrixX<double> J_x{2, 4};
  J_x << J_q, MatrixX<double>::Zero(1, 2), MatrixX<double>::Zero(1, 2), J_q;

  auto convert_to_x_tilde = builder.AddSystem<systems::MatrixGain<double>>(J_x);
  estimated_state_input_port_ =
      builder.ExportInput(convert_to_x_tilde->get_input_port());

  const Eigen::VectorXd wsg_kp = Eigen::VectorXd::Constant(J_q.rows(), 2000.0);
  const Eigen::VectorXd wsg_ki = Eigen::VectorXd::Constant(J_q.rows(), 0.0);
  const Eigen::VectorXd wsg_kd = Eigen::VectorXd::Constant(J_q.rows(), 5.0);

  auto wsg_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          wsg_kp, wsg_ki, wsg_kd);

  builder.Connect(convert_to_x_tilde->get_output_port(),
                  wsg_controller->get_input_port_estimated_state());
  builder.Connect(desired_mean_finger_state->get_output_port(),
                  wsg_controller->get_input_port_desired_state());

  // The PID controller outputs u_tilde. Add a matrix gain block to convert
  // to u.
  auto convert_to_u =
      builder.AddSystem<systems::MatrixGain<double>>(J_q.transpose());
  builder.Connect(wsg_controller->get_output_port_control(),
                  convert_to_u->get_input_port());
  auto adder = builder.AddSystem<systems::Adder<double>>(2, 2);
  builder.Connect(convert_to_u->get_output_port(), adder->get_input_port(0));
  auto convert_feed_forward_force_to_u =
      builder.AddSystem<systems::MatrixGain<double>>(
          (MatrixX<double>(2, 1) << 0.5, -0.4).finished());
  builder.Connect(convert_feed_forward_force_to_u->get_output_port(),
                  adder->get_input_port(1));
  feed_forward_force_input_port_ =
      builder.ExportInput(convert_feed_forward_force_to_u->get_input_port());
  builder.ExportOutput(adder->get_output_port());
  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
