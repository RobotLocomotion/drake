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
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>().
///
/// This system has one abstract-valued input port of type lcmt_iiwa_status.
///
/// This system has many vector-valued output ports, each of which has exactly
/// num_joints elements.  The ports will output zeros until an input message is
/// received.
//
/// @system
/// name: IiwaStatusReceiver
/// input_ports:
/// - lcmt_iiwa_status
/// output_ports:
/// - position_commanded
/// - position_measured
/// - velocity_estimated
/// - torque_commanded
/// - torque_measured
/// - torque_external
/// @endsystem
///
/// @see `lcmt_iiwa_status.lcm` for additional documentation.
class IiwaStatusReceiver final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStatusReceiver)

  explicit IiwaStatusReceiver(int num_joints = kIiwaArmNumJoints);
  ~IiwaStatusReceiver() final;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::OutputPort<double>& get_position_commanded_output_port() const;
  const systems::OutputPort<double>& get_position_measured_output_port() const;
  const systems::OutputPort<double>& get_velocity_estimated_output_port() const;
  const systems::OutputPort<double>& get_torque_commanded_output_port() const;
  const systems::OutputPort<double>& get_torque_measured_output_port() const;
  const systems::OutputPort<double>& get_torque_external_output_port() const;
  //@}

 private:
  template <std::vector<double> drake::lcmt_iiwa_status::*>
  void CalcLcmOutput(const systems::Context<double>&,
                     systems::BasicVector<double>*) const;

  const int num_joints_;
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
