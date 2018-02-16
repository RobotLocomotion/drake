#include "drake/manipulation/schunk_wsg/schunk_wsg_low_level_controller.h"

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

template <typename T>
SchunkWsgLowLevelController<T>::SchunkWsgLowLevelController(
    const Vector2<double>& closed_joint_position,
    const Vector2<double>& open_joint_position) {
  // Define position and state sizes.
  const int joint_position_size{2};  // Two fingers.
  const int joint_state_size{2 * joint_position_size};
  const int mean_finger_position_size{1};
  const int mean_finger_state_size{2 * mean_finger_position_size};
  const int grip_position_size{1};  // Distance between the fingers.
  const int grip_state_size{2 * grip_position_size};

  systems::DiagramBuilder<T> builder;

  // Define Jacobians for the mean finger position and the grip position
  // (distance between the fingers).
  const auto zero_mean_joint =
      MatrixX<double>::Zero(mean_finger_position_size, joint_position_size);
  const auto zero_grip_joint =
      MatrixX<double>::Zero(grip_position_size, joint_position_size);
  const MatrixX<double> grip_position_jacobian =
      (open_joint_position - closed_joint_position).transpose().cwiseSign();
  // clang-format off
  const auto mean_finger_position_jacobian =
      (MatrixX<double>(mean_finger_position_size, joint_position_size) <<
        0.5, 0.5
      ).finished();
  const auto mean_finger_state_jacobian =
      (MatrixX<double>(mean_finger_state_size, joint_state_size) <<
        mean_finger_position_jacobian, zero_mean_joint,
        zero_mean_joint,               mean_finger_position_jacobian
      ).finished();
  const auto grip_state_jacobian =
      (MatrixX<double>(grip_state_size, joint_state_size) <<
        grip_position_jacobian,      zero_grip_joint,
        zero_grip_joint,             grip_position_jacobian
      ).finished();
  // clang-format on

  // Add a pass through to provide the estimated state to multiple subsystems.
  const auto estimated_state_passthrough =
      builder.template AddSystem<systems::PassThrough<T>>(joint_state_size);

  // Add a constant source for the desired mean finger state (position and
  // velocity).
  // clang-format off
  const auto desired_mean_finger_state =
      builder.template AddSystem<systems::ConstantVectorSource<T>>(
          (VectorX<T>(mean_finger_state_size) <<
            closed_joint_position.mean(), 0.
          ).finished());
  // clang-format on

  // Add the PID controller.
  const Vector1d kp{2000.0};
  const Vector1d ki{0.0};
  const Vector1d kd{5.0};

  const auto pid_controller =
      builder.template AddSystem<systems::controllers::PidController<T>>(
          mean_finger_state_jacobian, mean_finger_position_jacobian.transpose(),
          kp, ki, kd);

  // Add a block to convert the commanded grip force to joint forces.
  const auto commanded_grip_force_to_joint_force =
      builder.template AddSystem<systems::MatrixGain<T>>(
          grip_position_jacobian.transpose());

  // Add a block to add the joint forces due to the PID controller and the
  // commanded grip force.
  const auto joint_force_adder =
      builder.template AddSystem<systems::Adder<T>>(2, joint_position_size);

  // Add a block to convert the joint state to the grip state.
  const auto joint_state_to_grip_state =
      builder.template AddSystem<systems::AffineSystem<T>>(
          MatrixX<double>::Zero(0, 0),                 // A
          MatrixX<double>::Zero(0, joint_state_size),  // B
          MatrixX<double>::Zero(0, 0),                 // f0
          MatrixX<double>::Zero(grip_state_size, 0),   // C
          grip_state_jacobian,                         // D
          closed_joint_position                        // y0
      );

  // Export the inputs.
  estimated_joint_state_input_port_ =
      builder.ExportInput(estimated_state_passthrough->get_input_port());
  commanded_grip_force_input_port_ = builder.ExportInput(
      commanded_grip_force_to_joint_force->get_input_port());

  // Export the outputs.
  estimated_grip_state_output_port_ =
      builder.ExportOutput(joint_state_to_grip_state->get_output_port());
  commanded_joint_force_output_port_ =
      builder.ExportOutput(joint_force_adder->get_output_port());

  // Connect the subsystems.
  // Estimated state -> other subsystems
  builder.Connect(estimated_state_passthrough->get_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(estimated_state_passthrough->get_output_port(),
                  joint_state_to_grip_state->get_input_port());
  // Desired mean finger state -> PID
  builder.Connect(desired_mean_finger_state->get_output_port(),
                  pid_controller->get_input_port_desired_state());
  // Other subsystems -> joint_force_adder
  builder.Connect(pid_controller->get_output_port_control(),
                  joint_force_adder->get_input_port(0));
  builder.Connect(commanded_grip_force_to_joint_force->get_output_port(),
                  joint_force_adder->get_input_port(1));

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::manipulation::schunk_wsg::SchunkWsgLowLevelController);
