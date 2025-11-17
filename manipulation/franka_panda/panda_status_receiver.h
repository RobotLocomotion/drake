#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

/// Handles lcmt_panda_status messages from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// LcmSubscriberSystem::Make<lcmt_panda_status>().
///
/// This system has one abstract-valued input port of type lcmt_panda_status.
///
/// This system has many vector-valued output ports, each of which has exactly
/// num_joints elements.  The ports will output zeros until an input message is
/// received.
//
/// @system
/// name: PandaStatusReceiver
/// input_ports:
/// - lcmt_panda_status
/// output_ports:
/// - position_commanded
/// - position_measured
/// - velocity_commanded
/// - velocity_measured
/// - acceleration_commanded
/// - torque_commanded
/// - torque_measured
/// - torque_external
/// @endsystem
///
/// @see `lcmt_panda_status.lcm` for additional documentation.
class PandaStatusReceiver final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PandaStatusReceiver);

  explicit PandaStatusReceiver(int num_joints = kPandaArmNumJoints);
  ~PandaStatusReceiver() final;

  using OutputPort = drake::systems::OutputPort<double>;
  const OutputPort& get_position_commanded_output_port() const;
  const OutputPort& get_position_measured_output_port() const;
  const OutputPort& get_velocity_commanded_output_port() const;
  const OutputPort& get_velocity_measured_output_port() const;
  const OutputPort& get_acceleration_commanded_output_port() const;
  const OutputPort& get_torque_commanded_output_port() const;
  const OutputPort& get_torque_measured_output_port() const;
  const OutputPort& get_torque_external_output_port() const;

 private:
  template <std::vector<double> drake::lcmt_panda_status::*>
  void CalcLcmOutput(const drake::systems::Context<double>&,
                     drake::systems::BasicVector<double>*) const;

  const int num_joints_;
};

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
