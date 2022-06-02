#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/drake_visualizer_config.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace multibody {

void ApplyDrakeVisualizerConfig(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm = nullptr,
    const DrakeVisualizerConfig& config = {});

}  // namespace multibody
}  // namespace drake
