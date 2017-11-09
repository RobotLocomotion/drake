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

using Eigen::Matrix;

namespace drake {
namespace manipulation {
namespace schunk_wsg {

SchunkWsgPlainController::SchunkWsgPlainController(ControlMode control_mode,
                                                   double kp, double ki,
                                                   double kd) {
  systems::DiagramBuilder<double> builder;

  // Define position and state sizes.
  constexpr int joint_position_size{2};  // Two fingers.
  constexpr int joint_state_size{2 * joint_position_size};
  constexpr int mean_finger_position_size{1};
  constexpr int grip_position_size{1};  // Distance between the fingers.

  // Define Jacobians for the mean finger position and the grip position
  // (distance between the fingers).
  // clang-format off
  const auto grip_position_jacobian =
      (Matrix<double, grip_position_size, joint_position_size>() <<
        0.5, -0.5)
      .finished();
  const auto mean_finger_position_jacobian =
      (Matrix<double, mean_finger_position_size, joint_position_size>() <<
        0.5, 0.5)
      .finished();
  // clang-format on

  // Add blocks to convert the estimated joint state to the estimated
  // controller state and the controller output to joint force.
  int control_position_size{};
  MatrixX<double> control_position_jacobian;
  switch (control_mode) {
    case ControlMode::kPosition: {
      control_position_size = grip_position_size + mean_finger_position_size;
      control_position_jacobian =
          (MatrixX<double>(control_position_size, joint_position_size)
               << grip_position_jacobian,
           mean_finger_position_jacobian)
              .finished();
      break;
    }
    case ControlMode::kForce: {
      control_position_size = mean_finger_position_size;
      control_position_jacobian =
          (MatrixX<double>(control_position_size, joint_position_size)
           << mean_finger_position_jacobian)
              .finished();
    }
  }
  const int control_state_size{2 * control_position_size};
  const auto zero_control =
      MatrixX<double>::Zero(control_position_size, joint_position_size);
  // clang-format off
  const auto control_state_jacobian =
      (MatrixX<double>(control_state_size, joint_state_size) <<
        control_position_jacobian, zero_control,
        zero_control,              control_position_jacobian)
      .finished();
  // clang-format on
  const auto joint_state_to_control_state =
      builder.AddSystem<systems::MatrixGain<double>>(control_state_jacobian);
  const auto controller_output_to_joint_force =
      builder.AddSystem<systems::MatrixGain<double>>(
          control_position_jacobian.transpose());

  // Add the PID controller
  const auto pid_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          VectorX<double>::Constant(control_position_size, kp),
          VectorX<double>::Constant(control_position_size, ki),
          VectorX<double>::Constant(control_position_size, kd));

  // Add the saturation block.
  // Create gain blocks to convert the scalar, positive max-force input into
  // positive and negative max-joint-force vectors.
  const auto max_force_passthrough =
      builder.AddSystem<systems::PassThrough<double>>(1);
  const auto positive_gain = builder.AddSystem<systems::MatrixGain<double>>(
      MatrixX<double>::Ones(joint_position_size, 1));
  const auto negative_gain = builder.AddSystem<systems::MatrixGain<double>>(
      -MatrixX<double>::Ones(joint_position_size, 1));
  const auto saturation = builder.AddSystem<systems::Saturation<double>>(2);

  // Add blocks to generate the desired control state.
  const systems::InputPortDescriptor<double>* desired_grip_state_input_port{};
  const systems::OutputPort<double>* desired_control_state_output_port{};
  // Add a source for the desired mean finger state. The mean finger
  // position and velocity should be zero.
  auto desired_mean_finger_state =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Vector2<double>::Zero());
  switch (control_mode) {
    case ControlMode::kPosition: {
      // Add a multiplexer to concatenate the desired grip state and the desired
      // mean finger state.
      auto concatenate_desired_states =
          builder.AddSystem<systems::Multiplexer<double>>(
              std::vector<int>({2, 2}));

      // The output of concatenate_desired_states has the form
      //
      //   [q_tilde_0, v_tilde_0, q_tilde_1, v_tilde_1].
      //
      // whereas the PID controller takes
      //
      //   [q_tilde_0, q_tilde_0, v_tilde_0, v_tilde_1]
      //
      // We now construct a matrix gain block to swap the order of th
      // elements.
      Matrix4<double> D;
      // clang-format off
      D << 1, 0, 0, 0,
           0, 0, 1, 0,
           0, 1, 0, 0,
           0, 0, 0, 1;
      // clang-format on
      auto convert_to_x_tilde_desired =
          builder.AddSystem<systems::MatrixGain<double>>(D);

      // Get the input port.
      desired_grip_state_input_port =
          &concatenate_desired_states->get_input_port(0);

      // Get the output port.
      desired_control_state_output_port =
          &convert_to_x_tilde_desired->get_output_port();

      // Connect the subsystems.
      builder.Connect(desired_mean_finger_state->get_output_port(),
                      concatenate_desired_states->get_input_port(1));
      builder.Connect(concatenate_desired_states->get_output_port(0),
                      convert_to_x_tilde_desired->get_input_port());
    } break;
    case ControlMode::kForce: {
      // Get the output port.
      desired_control_state_output_port =
          &desired_mean_finger_state->get_output_port();
    } break;
  }

  // Add blocks to handle the feed-forward force
  const systems::InputPortDescriptor<double>* feed_forward_force_input_port{};
  const systems::OutputPort<double>* joint_force_output_port{};
  switch (control_mode) {
    case ControlMode::kPosition: {
      // Get the output port.
      joint_force_output_port =
          &controller_output_to_joint_force->get_output_port();
    } break;
    case ControlMode::kForce: {
      // Add  block to convert the feed-forward force to joint force.
      auto grip_force_to_joint_force =
          builder.AddSystem<systems::MatrixGain<double>>(
              grip_position_jacobian.transpose());
      // Add a block to sum the joint forces from the PID controller with those
      // from the feed-forward force.
      auto adder = builder.AddSystem<systems::Adder<double>>(2, 2);

      // Get the input port.
      feed_forward_force_input_port =
          &grip_force_to_joint_force->get_input_port();

      // Get the output port.
      joint_force_output_port = &adder->get_output_port();

      // Connect the subsystems.
      builder.Connect(controller_output_to_joint_force->get_output_port(),
                      adder->get_input_port(0));
      builder.Connect(grip_force_to_joint_force->get_output_port(),
                      adder->get_input_port(1));
    } break;
  }

  // Export the inputs.
  state_input_port_ =
      builder.ExportInput(joint_state_to_control_state->get_input_port());
  max_force_input_port_ =
      builder.ExportInput(max_force_passthrough->get_input_port());
  if (desired_grip_state_input_port) {
    desired_grip_state_input_port_ =
        builder.ExportInput(*desired_grip_state_input_port);
  }
  if (feed_forward_force_input_port) {
    feed_forward_force_input_port_ =
        builder.ExportInput(*feed_forward_force_input_port);
  }

  // Export the output.
  builder.ExportOutput(saturation->get_output_port());

  // Connect the subsystems.
  // Upstream subsystems -> PID
  builder.Connect(joint_state_to_control_state->get_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(*desired_control_state_output_port,
                  pid_controller->get_input_port_desired_state());
  // PID -> Controller Output To Joint Force
  builder.Connect(pid_controller->get_output_port_control(),
                  controller_output_to_joint_force->get_input_port());
  // Joint force -> Saturation
  builder.Connect(*joint_force_output_port, saturation->get_input_port());
  // Max force input -> Saturation
  builder.Connect(max_force_passthrough->get_output_port(),
                  positive_gain->get_input_port());
  builder.Connect(max_force_passthrough->get_output_port(),
                  negative_gain->get_input_port());
  builder.Connect(positive_gain->get_output_port(),
                  saturation->get_max_value_port());
  builder.Connect(negative_gain->get_output_port(),
                  saturation->get_min_value_port());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
