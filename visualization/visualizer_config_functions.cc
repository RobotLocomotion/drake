#include "sim/common/drake_visualizer_config_functions.h"

#include <stdexcept>
#include <string>

#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"

using drake::geometry::DrakeVisualizer;
using drake::geometry::DrakeVisualizerParams;
using drake::geometry::Rgba;
using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcmInterface;
using drake::multibody::ConnectContactResultsToDrakeVisualizer;
using drake::multibody::MultibodyPlant;
using drake::systems::lcm::LcmBuses;

namespace anzu {
namespace sim {

void ApplyDrakeVisualizerConfig(
    const DrakeVisualizerConfig& config,
    const MultibodyPlant<double>& plant,
    const SceneGraph<double>& scene_graph,
    const LcmBuses& lcm_buses,
    drake::systems::DiagramBuilder<double>* builder) {
  // This is required due to BuildContactsPublisher().
  DRAKE_THROW_UNLESS(plant.is_finalized());
  DrakeLcmInterface* lcm = lcm_buses.Find("DrakeVisualizer", config.lcm_bus);
  const std::vector<DrakeVisualizerParams> all_params =
      internal::ConvertDrakeVisualizerConfigToParams(config);
  for (const DrakeVisualizerParams& params : all_params) {
    DrakeVisualizer<double>::AddToBuilder(builder, scene_graph, lcm, params);
  }
  if (config.publish_contacts) {
    ConnectContactResultsToDrakeVisualizer(builder, plant, scene_graph, lcm);
  }
}

namespace {

// TODO(jeremy.nimmer) Add a `Serialize()` function to `class Rgba` so that it
// can be serialized directly, without this helper function.
Rgba VectorToRgba(const Eigen::VectorXd& input) {
  if (input.size() < 3) {
    throw std::runtime_error("Rgba must have >= 3 values");
  }
  if (input.size() > 4) {
    throw std::runtime_error("Rgba must have <= 4 values");
  }
  const double r = input(0);
  const double g = input(1);
  const double b = input(2);
  const double a = (input.size() == 4) ? input(3) : 1.0;
  return Rgba(r, g, b, a);
}

}  // namespace

namespace internal {

std::vector<DrakeVisualizerParams>
ConvertDrakeVisualizerConfigToParams(
    const DrakeVisualizerConfig& config) {
  std::vector<DrakeVisualizerParams> result;

  if (config.publish_illustration) {
    DrakeVisualizerParams illustration;
    illustration.role = Role::kIllustration;
    illustration.publish_period = config.publish_period;
    illustration.default_color =
        VectorToRgba(config.default_illustration_color_rgba);
    result.push_back(illustration);
  }

  if (config.publish_proximity) {
    DrakeVisualizerParams proximity;
    proximity.role = Role::kProximity;
    proximity.publish_period = config.publish_period;
    proximity.default_color =
        VectorToRgba(config.default_proximity_color_rgba);
    proximity.show_hydroelastic = true;
    proximity.use_role_channel_suffix = true;
    result.push_back(proximity);
  }

  return result;
}

}  // namespace internal
}  // namespace sim
}  // namespace anzu
