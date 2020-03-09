#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_hsr_sim_command.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace hsr {

/// Receives a lcmt_hsr_sim_command message and outputs the desired state.
///
/// This system does not actually subscribe the message from an LCM channel. To
/// use this class, the input of this system should be connected to a
/// systems::lcm::LcmSubscriberSystem::Make<lcmt_hsr_sim_command>().
///
/// This system takes a const MultibodyPlant<double> pointer as the input
/// argument. It will be used to decide sizes of the the output port and to
/// parse the input port message.
///
/// @system {
///   @input_port{lcmt_hsr_sim_command}
///   @output_port{desired_state}
/// }
///
/// @see `lcmt_hsr_sim_command.lcm` for additional documentation.

class SimCommandReceiver final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimCommandReceiver)
  /// @p robot_plant is aliased and must remain valid for the lifetime of the
  /// command receiver.
  explicit SimCommandReceiver(
      const multibody::MultibodyPlant<double>* robot_plant);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_sim_command_input_port() const {
    return systems::LeafSystem<double>::get_input_port(0);
  }

  const systems::OutputPort<double>& get_desired_state_output_port() const {
    DRAKE_DEMAND(output_port_desired_state_ != nullptr);
    return *output_port_desired_state_;
  }
  //@}

 private:
  void CalcDesiredStateOutput(const systems::Context<double>& context,
                              systems::BasicVector<double>*) const;

  /// Internal reference to the robot plant.
  const multibody::MultibodyPlant<double>* const robot_plant_;

  const systems::OutputPort<double>* output_port_desired_state_{};
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
