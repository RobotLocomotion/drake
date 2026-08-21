#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

/// Creates and outputs lcmt_panda_status messages.
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// LcmPublisherSystem::Make<lcmt_panda_status>().
///
/// This system has many vector-valued input ports, each of which has exactly
/// num_joints elements.
///
/// This system has one abstract-valued output port of type lcmt_panda_status.
///
/// This system is presently only used in simulation. The robot hardware drivers
/// publish directly to LCM and do not make use of this system.
///
/// @system
/// name: PandaStatusSender
/// input_ports:
/// - position_commanded (optional)
/// - position_measured
/// - velocity_commanded (optional)
/// - velocity_measured (optional)
/// - acceleration_commanded (optional)
/// - torque_commanded
/// - torque_measured (optional)
/// - torque_external (optional)
/// output_ports:
/// - lcmt_panda_status
/// @endsystem
///
/// @see `lcmt_panda_status.lcm` for additional documentation.
class PandaStatusSender final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PandaStatusSender);

  explicit PandaStatusSender(int num_joints = kPandaArmNumJoints);
  ~PandaStatusSender() final;

  using InputPort = drake::systems::InputPort<double>;
  const InputPort& get_position_commanded_input_port() const;
  const InputPort& get_position_measured_input_port() const;
  const InputPort& get_velocity_commanded_input_port() const;
  const InputPort& get_velocity_measured_input_port() const;
  const InputPort& get_acceleration_commanded_input_port() const;
  const InputPort& get_torque_commanded_input_port() const;
  const InputPort& get_torque_measured_input_port() const;
  const InputPort& get_torque_external_input_port() const;

 private:
  void CalcOutput(const drake::systems::Context<double>&,
                  drake::lcmt_panda_status*) const;

  const int num_joints_;
  const Eigen::VectorXd zero_vector_;
};

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
