#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/// Handles IIWA commands from a LcmSubscriberSystem.
///
/// It has two input ports: the "command_message" for lcmt_iiwa_command abstract
/// values, and the "command_vector" for IiwaCommand. Only one of the inputs
/// should be connected. However, if both are connected, the message port will
/// be ignored. The "command_vector" port is deprecated and will be removed on
/// 2019-03-01.
///
/// It has two output ports: one for the commanded position for each joint along
/// with an estimate of the commanded velocity for each joint, and another for
/// commanded additional feedforward joint torque.
///
/// @system {
///   @input_port{command_message}
///   @input_port{command_vector (deprecated)}
///   @output_port{state}
///   @output_port{feedforward_torque}
/// }
class IiwaCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandReceiver)

  explicit IiwaCommandReceiver(int num_joints = kIiwaArmNumJoints);

  /// Sets the initial position of the controlled iiwa prior to any
  /// commands being received.  @p x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the starting position will be the zero
  /// configuration.
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const VectorX<double>> x) const;

  const systems::OutputPort<double>& get_commanded_state_output_port()
      const {
    return this->GetOutputPort("state");
  }

  const systems::OutputPort<double>& get_commanded_torque_output_port()
      const {
    return this->GetOutputPort("feedforward_torque");
  }

 private:
  void CopyStateToOutput(const systems::Context<double>& context, int start_idx,
                         int length,
                         systems::BasicVector<double>* output) const;

  // TODO(russt): This system should NOT have any state.
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  const int num_joints_;
};

/// Creates a LcmSubscriberSystem for lcmt_iiwa_command, using the fixed-size
/// message optimization; see LcmSubscriberSystem::MakeFixedSize for details.
std::unique_ptr<systems::lcm::LcmSubscriberSystem>
MakeIiwaCommandLcmSubscriberSystem(
    int num_joints, const std::string& channel,
    drake::lcm::DrakeLcmInterface* lcm);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
