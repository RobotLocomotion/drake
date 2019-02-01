#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/// Handles lcmt_iiwa_status messages from a LcmSubscriberSystem.
///
/// @system{ IiwaStatusReceiver,
///   @input_port{lcmt_iiwa_status},
///   @output_port{position_commanded}
///   @output_port{position_measured}
///   @output_port{velocity_estimated}
///   @output_port{torque_commanded}
///   @output_port{torque_measured}
///   @output_port{torque_external} }
///
/// All ports will output all zeros until a message is received.
///
/// @see `lcmt_iiwa_status.lcm` for additional documentation.
class IiwaStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStatusReceiver)

  explicit IiwaStatusReceiver(int num_joints = kIiwaArmNumJoints);

  const systems::OutputPort<double>& get_position_commanded_output_port()
  const {
    return this->get_output_port(position_commanded_output_port_);
  }

  const systems::OutputPort<double>& get_position_measured_output_port()
  const {
    return this->get_output_port(position_measured_output_port_);
  }

  const systems::OutputPort<double>& get_velocity_estimated_output_port()
  const {
    return this->get_output_port(velocity_estimated_output_port_);
  }

  const systems::OutputPort<double>& get_torque_commanded_output_port()
  const {
    return this->get_output_port(torque_commanded_output_port_);
  }

  const systems::OutputPort<double>& get_torque_measured_output_port()
  const {
    return this->get_output_port(torque_measured_output_port_);
  }

  const systems::OutputPort<double>& get_torque_external_output_port()
  const {
    return this->get_output_port(torque_external_output_port_);
  }

  const systems::OutputPort<double>& get_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

 private:
  template <std::vector<double> drake::lcmt_iiwa_status::* field>
  void CopyLcmVectorOut(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const;

  void OutputState(const systems::Context<double>& context,
                   systems::BasicVector<double>* output) const;

  const int num_joints_;

  const int position_commanded_output_port_{};
  const int position_measured_output_port_{};
  const int velocity_estimated_output_port_{};
  const int torque_commanded_output_port_{};
  const int torque_measured_output_port_{};
  const int torque_external_output_port_{};
  const int state_output_port_{};
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
