#include "drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h"

#include <utility>
#include <vector>

#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/systems/primitives/selector.h"

using Eigen::Matrix;

namespace drake {
namespace manipulation {
namespace schunk_wsg {

SchunkWsgPlainController::SchunkWsgPlainController(ControlMode control_mode,
                                                   double kp, double ki,
                                                   double kd) {
  systems::DiagramBuilder<double> builder;

  // Define position and state sizes.
  // Size of joint position vector, q. The gripper has two prismatic joints.
  // When the gripper is closed, q == 0. When the gripper is open, q[0] < 0 and
  // q[1] > 0.
  constexpr int nq{2};
  // Size of state (joint positions and velocities) vector, x.
  constexpr int nx{2 * nq};
  // Size of mean finger position representation.
  // The mean finger position, m, is a scalar given by
  //
  //    m = 0.5 (q[0] + q[1]).
  //
  // m == 0 implies that the fingers are centered.
  constexpr int nm{1};

  // Size of grip position representation.
  // The grip position, g, is a scalar given by
  //
  //    g = q[0] - q[1].
  //
  // g is 0 when the gripper is closed and decreases as the gripper opens. This
  // definition is chosen so that a positive grip force corresponds to closing
  // the gripper.
  constexpr int ng{1};

  // Define Jacobians for the mean finger position and the grip position
  // Mean finger position Jacobian (nm × nq).
  const auto Jm = (Matrix<double, nm, nq>() << 0.5, 0.5).finished();

  // Grip position Jacobian (ng × nq).
  const auto Jg = (Matrix<double, ng, nq>() << 1, -1).finished();

  // Add blocks to convert the estimated joint state to the estimated
  // controller state and the controller output to joint force.

  // Size of the control position, c. The control position is the quantity
  // regulated by the PID controller. The size of the control position vector
  // depends on which control mode is used.
  int nc{};
  // Control position Jacobian (nc × nq).
  MatrixX<double> Jc;
  switch (control_mode) {
    case ControlMode::kPosition: {
      nc = nm + ng;
      Jc = (MatrixX<double>(nc, nq) << Jm, Jg).finished();
      break;
    }
    case ControlMode::kForce: {
      nc = nm;
      Jc = (MatrixX<double>(nc, nq) << Jm).finished();
    }
  }
  const auto zero_nc_by_nq = MatrixX<double>::Zero(nc, nq);
  // clang-format off
  const auto control_state_jacobian =
      (MatrixX<double>(2 * nc, nx) <<
        Jc,             zero_nc_by_nq,
        zero_nc_by_nq,  Jc)
      .finished();
  // clang-format on
  const auto joint_state_to_control_state =
      builder.AddSystem<systems::MatrixGain<double>>(control_state_jacobian);

  // Add the PID controller
  const auto pid_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          VectorX<double>::Constant(nc, kp), VectorX<double>::Constant(nc, ki),
          VectorX<double>::Constant(nc, kd));

  // Add the saturation block.
  // Create gain blocks to convert the scalar, positive max-force input into
  // positive and negative max-joint-force vectors.
  const auto positive_gain = builder.AddSystem<systems::MatrixGain<double>>(
      0.5 * MatrixX<double>::Ones(ng, 1));
  const auto negative_gain = builder.AddSystem<systems::MatrixGain<double>>(
      -0.5 * MatrixX<double>::Ones(ng, 1));
  const auto saturation = builder.AddSystem<systems::Saturation<double>>(ng);

  // Add blocks to generate the desired control state.
  const systems::InputPort<double>* desired_grip_state_input_port{};
  const systems::OutputPort<double>* desired_control_state_output_port{};
  // Add a source for the desired mean finger state. The mean finger
  // position and velocity should be zero.
  auto desired_mean_finger_state =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Vector2<double>::Zero());
  switch (control_mode) {
    case ControlMode::kPosition: {
      // Implement the following recipe:
      //  x_tilde_desired = [q_tilde_0, q_tilde_1, v_tilde_0, v_tilde_1]
      // with
      //  q_tilde_0 = desired_mean_finger_state[0]
      //  v_tilde_0 = desired_mean_finger_state[1]
      //  q_tilde_1 = desired_grip_state[0]
      //  v_tilde_1 = desired_grip_state[1]
      // using a selector system.
      const systems::SelectorParams params{
          .inputs{systems::SelectorParams::InputPortParams{
                      .name = "desired_mean_finger_state",
                      .size = 2,
                  },
                  systems::SelectorParams::InputPortParams{
                      .name = "desired_grip_state",
                      .size = 2,
                  }},
          .outputs{systems::SelectorParams::OutputPortParams{
              .name = "x_tilde_desired",
              .selections{
                  {
                      .input_port_index = 0,
                      .input_offset = 0,
                  },
                  {
                      .input_port_index = 1,
                      .input_offset = 0,
                  },
                  {
                      .input_port_index = 0,
                      .input_offset = 1,
                  },
                  {
                      .input_port_index = 1,
                      .input_offset = 1,
                  },
              }}}};
      auto x_tilde_desired =
          builder.AddSystem<systems::Selector<double>>(std::move(params));
      builder.Connect(desired_mean_finger_state->get_output_port(),
                      x_tilde_desired->get_input_port(0));

      // Get the input port.
      desired_grip_state_input_port = &x_tilde_desired->get_input_port(1);

      // Get the output port.
      desired_control_state_output_port = &x_tilde_desired->get_output_port();
      break;
    }
    case ControlMode::kForce: {
      // Get the output port.
      desired_control_state_output_port =
          &desired_mean_finger_state->get_output_port();
      break;
    }
  }

  // Add blocks to handle the feed-forward force
  const systems::InputPort<double>* feed_forward_force_input_port{};
  const systems::OutputPort<double>* mean_finger_force_output_port{};
  const systems::OutputPort<double>* grip_force_output_port{};
  switch (control_mode) {
    case ControlMode::kPosition: {
      // Add a de-multiplexer to split the PID outputs
      auto demux = builder.AddSystem<systems::Demultiplexer<double>>(nc);

      // Get the output ports.
      mean_finger_force_output_port = &demux->get_output_port(0);
      grip_force_output_port = &demux->get_output_port(1);

      // Connect the subsystems.
      builder.Connect(pid_controller->get_output_port_control(),
                      demux->get_input_port(0));
      break;
    }
    case ControlMode::kForce: {
      // Add a pass-through for the feed-forward force.
      auto pass_through = builder.AddSystem<systems::PassThrough<double>>(ng);

      // Get the input port.
      feed_forward_force_input_port = &pass_through->get_input_port();

      // Get the output ports.
      mean_finger_force_output_port =
          &pid_controller->get_output_port_control();
      grip_force_output_port = &pass_through->get_output_port();

      break;
    }
  }

  // Add block to convert the grip force to joint force.
  auto grip_force_to_joint_force =
      builder.AddSystem<systems::MatrixGain<double>>(Jg.transpose());

  // Add block to convert the mean_finger force to joint force.
  auto mean_finger_force_to_joint_force =
      builder.AddSystem<systems::MatrixGain<double>>(Jm.transpose());

  // Add a block to sum the joint forces from the mean finger force and grip
  // force.
  auto adder = builder.AddSystem<systems::Adder<double>>(2 /*num_inputs*/, nq);

  // Export the inputs.
  state_input_port_ = builder.ExportInput(
      joint_state_to_control_state->get_input_port(), "joint_state");
  // Max force input -> Saturation
  max_force_input_port_ =
      builder.ExportInput(positive_gain->get_input_port(), "max_force");
  builder.ConnectInput(max_force_input_port_, negative_gain->get_input_port());
  if (desired_grip_state_input_port) {
    desired_grip_state_input_port_ = builder.ExportInput(
        *desired_grip_state_input_port, "desired_grip_state");
  }
  if (feed_forward_force_input_port) {
    feed_forward_force_input_port_ = builder.ExportInput(
        *feed_forward_force_input_port, "feed_forward_force");
  }

  // Export the output.
  builder.ExportOutput(adder->get_output_port(), "control");

  // Connect the subsystems.
  // Upstream subsystems -> PID
  builder.Connect(joint_state_to_control_state->get_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(*desired_control_state_output_port,
                  pid_controller->get_input_port_desired_state());
  // Mean finger force -> Mean Finger Force to Joint Force
  builder.Connect(*mean_finger_force_output_port,
                  mean_finger_force_to_joint_force->get_input_port());
  // Grip force -> Saturation
  builder.Connect(*grip_force_output_port, saturation->get_input_port());
  builder.Connect(positive_gain->get_output_port(),
                  saturation->get_max_value_port());
  builder.Connect(negative_gain->get_output_port(),
                  saturation->get_min_value_port());
  // Saturation -> Grip Force To Joint Force
  builder.Connect(saturation->get_output_port(),
                  grip_force_to_joint_force->get_input_port());
  // Joint forces -> Adder
  builder.Connect(mean_finger_force_to_joint_force->get_output_port(),
                  adder->get_input_port(0));
  builder.Connect(grip_force_to_joint_force->get_output_port(),
                  adder->get_input_port(1));

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
