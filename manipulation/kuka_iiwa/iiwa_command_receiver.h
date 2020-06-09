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
/// Before receiving a valid lcmt_iiwa_command message, the "position" output
/// will be set to a fallback value, and the "torque" output will always be a
/// vector of zeros. The fallback value can be set by connecting the
/// "position_measured" input port. This system uses DiscreteUpdate
/// to initialize the fallback value with the "position_measured" input port at
/// the first tick. When used with a Simulator, this is handled automatically
/// by the framework. When "position_measured" is not connected, the fallback
/// value will be initialized to zeros.
///
/// It has two output ports: one for the commanded position for each joint, and
/// one for commanded additional feedforward joint torque.
///
/// @system { IiwaCommandReceiver,
///   @input_port{lcmt_iiwa_command}
///   @input_port{position_measured}
///   @output_port{position}
///   @output_port{torque}
/// }
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

  /// XXX
  void LatchInitialPosition(systems::Context<double>* context) const;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_message_input_port() const;
  const systems::InputPort<double>& get_position_measured_input_port() const;
  const systems::OutputPort<double>& get_commanded_position_output_port() const;
  const systems::OutputPort<double>& get_commanded_torque_output_port() const;
  //@}

  DRAKE_DEPRECATED("2020-09-01", "Use get_message_input_port() instead.")
  const systems::InputPort<double>& get_input_port() const {
    return get_message_input_port();
  }

 private:
  void DoCalcNextUpdateTime(
      const systems::Context<double>&,
      systems::CompositeEventCollection<double>*, double*) const override;
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
