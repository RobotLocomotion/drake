#pragma once

#include <optional>

#include <Eigen/Dense>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// Builds (into @p builder) the WSG control and sensing systems for the wsg
/// model in @p plant indicated by @p wsg_instance; hooks those systems up to
/// @p lcm and the relevant MultibodyPlant ports in the diagram. @p pid_gains
/// can be used to override the default sim PID gains.
void BuildSchunkWsgControl(
    const multibody::MultibodyPlant<double>& plant,
    multibody::ModelInstanceIndex wsg_instance, lcm::DrakeLcmInterface* lcm,
    systems::DiagramBuilder<double>* builder,
    const std::optional<Eigen::Vector3d>& pid_gains = {});

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
