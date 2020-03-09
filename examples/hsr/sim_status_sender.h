#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_hsr_sim_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace hsr {

/// Creates and outputs lcmt_hsr_sim_status messages.
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// systems::lcm::LcmPublisherSystem::Make<lcmt_hsr_sim_status>().
///
/// This system has one vector-valued input port for the estimated state, which
/// must be connected. This system has one abstract-valued output port of
/// type lcmt_hsr_sim_status.
///
/// @system {
///   @input_port{estimated_state}
///   @output_port{lcmt_hsr_sim_status}
/// }
///
/// @see `lcmt_hsr_sim_status.lcm` for additional documentation.
class SimStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimStatusSender)

  /// @p robot_plant is aliased and must remain valid for the lifetime of the
  /// command sender.
  explicit SimStatusSender(
      const drake::multibody::MultibodyPlant<double>* robot_plant);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_estimated_state_input_port() const {
    return this->get_input_port(0);
  }
  const systems::OutputPort<double>& get_sim_status_output_port() const {
    return this->get_output_port(0);
  }
  //@}

 private:
  // Allocate and initialize the message.
  lcmt_hsr_sim_status MakeOutput() const;

  void CalcOutput(const systems::Context<double>&, lcmt_hsr_sim_status*) const;

  const drake::multibody::MultibodyPlant<double>* const robot_plant_;
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
