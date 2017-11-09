#include "drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h"

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

SchunkWsgPlainController::SchunkWsgPlainController(
    ControlMode control_mode) {
  systems::DiagramBuilder<double> builder;

  // Set up desired control state, x_d, signal.
  const systems::OutputPort<double>* x_d_port;

  // The mean finger position should be zero.
  auto desired_mean_finger_state =
    builder.AddSystem<systems::ConstantVectorSource<double>>(
        Vector2<double>::Zero());
  switch (control_mode) {
    case ControlMode::kPosition:
    case ControlMode::kPositionAndForce: {
      auto concatenate_desired_states =
          builder.AddSystem<systems::Multiplexer<double>>(
              std::vector<int>({2, 2}));

      desired_state_input_port_ =
          builder.ExportInput(concatenate_desired_states->get_input_port(0));
      builder.Connect(desired_mean_finger_state->get_output_port(),
                      concatenate_desired_states->get_input_port(1));

      // The output of concatenate_desired_states has the form
      //
      //   [q_tilde_0, v_tilde_0, q_tilde_1, v_tilde_1].
      //
      // whereas the PID controller takes
      //
      //   [q_tilde_0, q_tilde_0, v_tilde_0, v_tilde_1]
      //
      // We now construct a matrix gain block to swap the order of th elements.
      Matrix4<double> D;
      // clang-format off
      D << 1, 0, 0, 0,
           0, 0, 1, 0,
           0, 1, 0, 0,
           0, 0, 0, 1;
      // clang-format on
      auto convert_to_x_tilde_desired =
          builder.AddSystem<systems::MatrixGain<double>>(D);

      builder.Connect(concatenate_desired_states->get_output_port(0),
          convert_to_x_tilde_desired->get_input_port());

      x_d_port = &convert_to_x_tilde_desired->get_output_port();
    } break;
    case ControlMode::kForce: {
      x_d_port = &desired_mean_finger_state->get_output_port();
    } break;
  }

  // Add the PID controller.
  // Set up the pre-saturation control, u, signal.
  const systems::OutputPort<double>* u_port;
  switch (control_mode) {
    case ControlMode::kPosition:
    case ControlMode::kPositionAndForce: {
      //  q0 = (qR - qL)
      //  q1 = 0.5(qR + qL)
      //  q0d = command
      //  q1d = 0 (always)
      // Demux the gripper positions and velocities
      Matrix2<double> J_q;
      J_q << 0.5, -0.5, 0.5, 0.5;
      Matrix4<double> J_x;
      J_x << J_q, Matrix2<double>::Zero(), Matrix2<double>::Zero(), J_q;

      auto convert_to_x_tilde =
          builder.AddSystem<systems::MatrixGain<double>>(J_x);
      state_input_port_ =
          builder.ExportInput(convert_to_x_tilde->get_input_port());

      // The p gain here is somewhat arbitrary.  The goal is to make sure
      // that the maximum force is generated except when very close to the
      // target.
      const Eigen::VectorXd wsg_kp =
          Eigen::VectorXd::Constant(J_q.rows(), 2000.0);
      const Eigen::VectorXd wsg_ki = Eigen::VectorXd::Constant(J_q.rows(), 0.0);
      const Eigen::VectorXd wsg_kd = Eigen::VectorXd::Constant(J_q.rows(), 5.0);

      auto wsg_controller =
          builder.AddSystem<systems::controllers::PidController<double>>(
              wsg_kp, wsg_ki, wsg_kd);

      builder.Connect(convert_to_x_tilde->get_output_port(),
          wsg_controller->get_input_port_estimated_state());
      builder.Connect(*x_d_port,
                      wsg_controller->get_input_port_desired_state());

      // The PID controller outputs u_tilde. Add a matrix gain block to convert
      // to u.
      auto convert_to_u =
          builder.AddSystem<systems::MatrixGain<double>>(J_q.transpose());
      builder.Connect(wsg_controller->get_output_port_control(),
                      convert_to_u->get_input_port());

      u_port = &convert_to_u->get_output_port();
    } break;
    case ControlMode::kForce: {
      MatrixX<double> J_q{1, 2};
      J_q << 0.5, 0.5;
      MatrixX<double> J_x{2, 4};
      J_x << J_q, MatrixX<double>::Zero(1, 2), MatrixX<double>::Zero(1, 2), J_q;

      auto convert_to_x_tilde =
          builder.AddSystem<systems::MatrixGain<double>>(J_x);
      state_input_port_ =
          builder.ExportInput(convert_to_x_tilde->get_input_port());

      // The p gain here is somewhat arbitrary.  The goal is to make sure
      // that the maximum force is generated except when very close to the
      // target.
      const Eigen::VectorXd wsg_kp =
          Eigen::VectorXd::Constant(J_q.rows(), 2000.0);
      const Eigen::VectorXd wsg_ki = Eigen::VectorXd::Constant(J_q.rows(), 0.0);
      const Eigen::VectorXd wsg_kd = Eigen::VectorXd::Constant(J_q.rows(), 5.0);

      auto wsg_controller =
          builder.AddSystem<systems::controllers::PidController<double>>(
              wsg_kp, wsg_ki, wsg_kd);

      builder.Connect(convert_to_x_tilde->get_output_port(),
          wsg_controller->get_input_port_estimated_state());
      builder.Connect(*x_d_port,
                      wsg_controller->get_input_port_desired_state());

      // The PID controller outputs u_tilde. Add a matrix gain block to convert
      // to u.
      auto convert_to_u =
          builder.AddSystem<systems::MatrixGain<double>>(J_q.transpose());
      builder.Connect(wsg_controller->get_output_port_control(),
                      convert_to_u->get_input_port());
      auto adder = builder.AddSystem<systems::Adder<double>>(2, 2);
      builder.Connect(convert_to_u->get_output_port(),
                      adder->get_input_port(0));
      feed_forward_force_input_port_ =
          builder.ExportInput(adder->get_input_port(1));
      u_port = &adder->get_output_port();
    } break;
  }


  // Add the saturation block.
  // Create a gain block to negate the max force (to produce a minimum
  // force).
  auto max_force_passthrough =
      builder.AddSystem<systems::PassThrough<double>>(1);
  max_force_input_port_ =
      builder.ExportInput(max_force_passthrough->get_input_port());
  auto positive_gain = builder.AddSystem<systems::MatrixGain<double>>(
      MatrixX<double>::Ones(2, 1));
  auto negative_gain = builder.AddSystem<systems::MatrixGain<double>>(
      -MatrixX<double>::Ones(2, 1));
  builder.Connect(max_force_passthrough->get_output_port(),
                  positive_gain->get_input_port());
  builder.Connect(max_force_passthrough->get_output_port(),
                  negative_gain->get_input_port());

  auto saturation = builder.AddSystem<systems::Saturation<double>>(2);

  builder.Connect(*u_port, saturation->get_input_port());
  builder.Connect(positive_gain->get_output_port(),
                  saturation->get_max_value_port());
  builder.Connect(negative_gain->get_output_port(),
                  saturation->get_min_value_port());
  builder.ExportOutput(saturation->get_output_port());
  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
