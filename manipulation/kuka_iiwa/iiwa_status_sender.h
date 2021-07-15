#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/// Creates and outputs lcmt_iiwa_status messages.
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>().
///
/// This system has many vector-valued input ports, each of which has exactly
/// num_joints elements.
///
/// - `position_commanded`: the most recently received position command.
/// - `position_measured`: the plant's current position.
/// - `velocity_estimated` (optional): the plant's current velocity (this
///     should be a low-pass filter of the position's derivative; see detailed
///     comments in `lcmt_iiwa_status.lcm`); when absent, the output message
///     will use zeros.
/// - `torque_commanded`: the most recently received joint torque command.
/// - `torque_measured` (optional): the plant's measured joint torque; when
///     absent, the output message will duplicate torque_commanded.
/// - `torque_external` (optional): the plant's external joint torque; when
///     absent, the output message will use zeros.
///
/// This system has one abstract-valued output port of type lcmt_iiwa_status.
///
/// This system is presently only used in simulation. The robot hardware drivers
/// publish directly to LCM and do not make use of this system.
///
/// @system
/// name: IiwaStatusSender
/// input_ports:
/// - position_commanded
/// - position_measured
/// - velocity_estimated (optional)
/// - torque_commanded
/// - torque_measured (optional)
/// - torque_external (optional)
/// output_ports:
/// - lcmt_iiwa_status
/// @endsystem
///
/// @see `lcmt_iiwa_status.lcm` for additional documentation.
class IiwaStatusSender final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStatusSender)

  explicit IiwaStatusSender(int num_joints = kIiwaArmNumJoints);
  ~IiwaStatusSender() final;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_position_commanded_input_port() const;
  const systems::InputPort<double>& get_position_measured_input_port() const;
  const systems::InputPort<double>& get_velocity_estimated_input_port() const;
  const systems::InputPort<double>& get_torque_commanded_input_port() const;
  const systems::InputPort<double>& get_torque_measured_input_port() const;
  const systems::InputPort<double>& get_torque_external_input_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_iiwa_status*) const;

  const int num_joints_;
  const Eigen::VectorXd zero_vector_;
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
