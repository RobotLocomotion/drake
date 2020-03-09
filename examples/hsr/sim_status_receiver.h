#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_hsr_sim_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace hsr {

/// Receives a lcmt_hsr_sim_status message and outputs the estimated state and
/// the torque information.
///
/// This system does not actually subscribe the message from an LCM channel. To
/// use this class, the input of this system should be connected to a
/// systems::lcm::LcmSubscriberSystem::Make<lcmt_hsr_sim_status>().
///
/// This system takes a const MultibodyPlant<double> pointer as the input
/// argument. It will be used to decide sizes of the the output port and to
/// parse the input port message.
///
/// @system {
///   @input_port{lcmt_hsr_sim_status}
///   @output_port{estimated_state}
///   @output_port{torque}
/// }
///
/// @see `lcmt_hsr_sim_status.lcm` for additional documentation.
class SimStatusReceiver final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimStatusReceiver)
  /// @p robot_plant is aliased and must remain valid for the lifetime of the
  /// status receiver.
  explicit SimStatusReceiver(
      const multibody::MultibodyPlant<double>* robot_plant);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_sim_status_input_port() const {
    return systems::LeafSystem<double>::get_input_port(0);
  }

  const systems::OutputPort<double>& get_estimated_state_output_port() const {
    DRAKE_DEMAND(output_port_estimated_state_ != nullptr);
    return *output_port_estimated_state_;
  }

  const systems::OutputPort<double>& get_torque_output_port() const {
    DRAKE_DEMAND(output_port_torque_ != nullptr);
    return *output_port_torque_;
  }
  //@}

 private:
  void CalcEstimatedStateOutput(const systems::Context<double>& context,
                                systems::BasicVector<double>*) const;

  void CalcTorqueOutput(const systems::Context<double>& context,
                        systems::BasicVector<double>*) const;

  /// Internal reference to the robot plant.
  const multibody::MultibodyPlant<double>* const robot_plant_;

  const systems::OutputPort<double>* output_port_estimated_state_{};
  const systems::OutputPort<double>* output_port_torque_{};
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
