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
/// It has one input port, "lcmt_iiwa_command".
/// (As well as some deprecated input ports; see below.)
///
/// It has two output ports: one for the commanded position for each joint, and
/// one for commanded additional feedforward joint torque.
/// (As well as some deprecated output ports; see below.)
///
/// It also has one additional, deprecated input port -- "command_message" as a
/// synonym for "lcmt_iiwa_command".  The "command_message" port will be
/// removed on 2019-05-01.  Exactly one of the two inputs must be connected.
///
/// It also has one additional, deprecated output port -- "state" for the
/// commanded position AND an estimate of the commanded velocity for each
/// joint.
///
/// @system { IiwaCommandReceiver,
///   @input_port{lcmt_iiwa_command}
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
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const Eigen::VectorXd>& q) const;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_input_port() const;
  const systems::OutputPort<double>& get_commanded_position_output_port() const;
  const systems::OutputPort<double>& get_commanded_torque_output_port() const;
  //@}

#ifndef DRAKE_DOXYGEN_CXX
  DRAKE_DEPRECATED("2019-05-01",
      "The state port is deprecated. "
      "Instead, use the \"position\" port.")
  const systems::OutputPort<double>& get_commanded_state_output_port() const;
  DRAKE_DEPRECATED("2019-05-01",
      "Instead, use get_input_port() with no arguments.")
  // TODO(jwnimmer-tri) Change this to `= delete;` after deprecation expires.
  const systems::InputPort<double>& get_input_port(int index) const {
    return LeafSystem<double>::get_input_port(index);
  }
  DRAKE_DEPRECATED("2019-05-01",
      "Instead, use the named port accessors.")
  // TODO(jwnimmer-tri) Change this to `= delete;` after deprecation expires.
  const systems::OutputPort<double>& get_output_port(int index) const {
    return LeafSystem<double>::get_output_port(index);
  }
#endif  //  DRAKE_DOXYGEN_CXX

 private:
  using MapVectorXd = Eigen::Map<const Eigen::VectorXd>;
  MapVectorXd input_position(const systems::Context<double>&) const;
  MapVectorXd input_torque(const systems::Context<double>&) const;
  void CalcInput(const systems::Context<double>&, lcmt_iiwa_command*) const;
  void CalcStateUpdate(
      const systems::Context<double>&, systems::DiscreteValues<double>*) const;
  void CalcStateOutput(
      const systems::Context<double>&, systems::BasicVector<double>*) const;

  const int num_joints_;
  const systems::CacheEntry* groomed_input_{};
};

/// Creates a LcmSubscriberSystem for lcmt_iiwa_command.
DRAKE_DEPRECATED("2019-07-01",
    "Call LcmSubscriberSystem::Make<drake::lcmt_iiwa_command> instead.")
std::unique_ptr<systems::lcm::LcmSubscriberSystem>
MakeIiwaCommandLcmSubscriberSystem(
    int num_joints, const std::string& channel,
    drake::lcm::DrakeLcmInterface* lcm);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
