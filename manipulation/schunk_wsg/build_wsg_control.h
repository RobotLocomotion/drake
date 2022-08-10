#pragma once

#include <optional>

#include <Eigen/Dense>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace anzu {
namespace sim {

/// Builds (into @p builder) the WSG control and sensing systems for the wsg
/// model in @p plant indicated by @p wsg_instance; hooks those systems up to
/// @p lcm and the relevant MbP ports in the diagram.
/// The argument pid_gains can be used to override the default sim PID gains.
void BuildWsgControl(const drake::multibody::MultibodyPlant<double>& plant,
                     const drake::multibody::ModelInstanceIndex wsg_instance,
                     drake::lcm::DrakeLcmInterface* lcm,
                     drake::systems::DiagramBuilder<double>* builder,
                     const std::optional<Eigen::Vector3d>& pid_gains = {});

}  // namespace sim
}  // namespace anzu
