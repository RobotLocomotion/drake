#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/// Handles lcmt_iiwa_command message from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>().
///
/// It has one required input port, "lcmt_iiwa_command".
///
/// It has two output ports: one for the commanded position for each joint, and
/// one for commanded additional feedforward joint torque.
///
/// @system{IiwaCommandReceiver,
///   @input_port{lcmt_iiwa_command}
///   @input_port{position_measured (optional)},
///   @output_port{position}
///   @output_port{torque}
/// }
///
/// @par Output prior to receiving a valid lcmt_iiwa_command message:
/// The "position" output initially feeds through from the "position_measured"
/// input port -- or if not connected, outputs zero.  When discrete update
/// events are enabled (e.g., during a simulation), the system latches the
/// "position_measured" input into state during the first event, and the
/// "position" output comes from the latched state, no longer fed through from
/// the "position" input.  Alternatively, the LatchInitialPosition() method is
/// available to achieve the same effect without using events.
/// @par
/// The "torque" output will always be a vector of zeros.
class IiwaCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandReceiver)

  explicit IiwaCommandReceiver(int num_joints = kIiwaArmNumJoints);

  /// Sets the initial commanded position of the controlled iiwa prior to any
  /// command messages being received.  If this function is not called, the
  /// starting position will be the zero configuration.  The initial commanded
  /// torque is always zero and cannot be set.
  DRAKE_DEPRECATED("2020-09-01",
      "To provide position commands prior to receiving the first message, "
      "connect the position_measured instead of setting this parameter")
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const Eigen::VectorXd>& q) const;

  /// (Advanced.) Copies the current "position_measured" input (or zero if not
  /// connected) into Context state, and changes the behavior of the "position"
  /// output to produce the latched state if no message has been received yet.
  /// The latching already happens automatically during the first discrete
  /// update event (e.g., when using a Simulator); this method exists for use
  /// when not already using a Simulator or other special cases.
  void LatchInitialPosition(systems::Context<double>* context) const;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_message_input_port() const {
    return *message_input_;
  }
  const systems::InputPort<double>& get_position_measured_input_port() const {
    return *position_measured_input_;
  }
  const systems::OutputPort<double>& get_commanded_position_output_port()
      const {
    return *commanded_position_output_;
  }
  const systems::OutputPort<double>& get_commanded_torque_output_port() const {
    return *commanded_torque_output_;
  }
  //@}

  DRAKE_DEPRECATED("2020-09-01", "Use get_message_input_port() instead.")
  const systems::InputPort<double>& get_input_port() const {
    return get_message_input_port();
  }

 private:
  void DoCalcNextUpdateTime(
      const systems::Context<double>&,
      systems::CompositeEventCollection<double>*, double*) const override;

  void CalcPositionMeasuredOrParam(
      const systems::Context<double>&, systems::BasicVector<double>*) const;
  void LatchInitialPosition(
      const systems::Context<double>&,
      systems::DiscreteValues<double>*) const;
  void CalcDefaultedCommand(
      const systems::Context<double>&, lcmt_iiwa_command*) const;
  void CalcPositionOutput(
      const systems::Context<double>&, systems::BasicVector<double>*) const;
  void CalcTorqueOutput(
      const systems::Context<double>&, systems::BasicVector<double>*) const;

  const int num_joints_;
  const systems::InputPort<double>* message_input_{};
  const systems::InputPort<double>* position_measured_input_{};
  const systems::CacheEntry* position_measured_or_param_{};
  systems::DiscreteStateIndex latched_position_measured_is_set_;
  systems::DiscreteStateIndex latched_position_measured_;
  const systems::CacheEntry* defaulted_command_{};
  const systems::OutputPort<double>* commanded_position_output_{};
  const systems::OutputPort<double>* commanded_torque_output_{};

  // Deprecated.
  systems::NumericParameterIndex initial_state_param_;
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
