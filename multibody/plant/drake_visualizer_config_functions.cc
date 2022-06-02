#include "drake/multibody/plant/drake_visualizer_config_functions.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"

namespace drake {
namespace multibody {

void ApplyDrakeVisualizerConfig(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm,
    const DrakeVisualizerConfig& config) {
  geometry::DrakeVisualizerd::AddToBuilderForRoles(
      builder, scene_graph, lcm, config.geometry_config);
  if (config.show_contact_results) {
    ConnectContactResultsToDrakeVisualizer(
        builder, plant, scene_graph, lcm,
        config.geometry_config.publish_period);
  }
}

}  // namespace multibody
}  // namespace drake

