#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
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
///
/// It has two output ports: one for the commanded position for each joint, and
/// one for commanded additional feedforward joint torque.
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

 private:
  using MapVectorXd = Eigen::Map<const Eigen::VectorXd>;
  MapVectorXd input_position(const systems::Context<double>&) const;
  MapVectorXd input_torque(const systems::Context<double>&) const;
  void CalcInput(const systems::Context<double>&, lcmt_iiwa_command*) const;

  const int num_joints_;
  const systems::CacheEntry* groomed_input_{};
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
