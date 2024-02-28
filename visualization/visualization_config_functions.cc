#include "drake/visualization/visualization_config_functions.h"

#include <stdexcept>
#include <string>
#include <utility>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/lcm/lcm_config_functions.h"
#include "drake/visualization/inertia_visualizer.h"

namespace drake {
namespace visualization {
namespace {

using geometry::DrakeVisualizer;
using geometry::DrakeVisualizerParams;
using geometry::MeshcatVisualizer;
using geometry::MeshcatVisualizerParams;
using geometry::Rgba;
using geometry::Role;
using geometry::SceneGraph;
using lcm::DrakeLcmInterface;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::MultibodyPlant;
using multibody::meshcat::ContactVisualizer;
using multibody::meshcat::ContactVisualizerParams;
using systems::DiagramBuilder;
using systems::System;
using systems::lcm::LcmBuses;

void ApplyVisualizationConfigImpl(const VisualizationConfig& config,
                                  DrakeLcmInterface* lcm,
                                  std::shared_ptr<geometry::Meshcat> meshcat,
                                  const MultibodyPlant<double>& plant,
                                  SceneGraph<double>* scene_graph,
                                  DiagramBuilder<double>* builder) {
  DRAKE_DEMAND(lcm != nullptr);
  DRAKE_DEMAND(scene_graph != nullptr);
  DRAKE_DEMAND(builder != nullptr);

  // This is required due to ConnectContactResultsToDrakeVisualizer().
  DRAKE_THROW_UNLESS(plant.is_finalized());

  // Add the LCM-based visualization, unless the user has opted out.
  if (lcm->get_lcm_url() != LcmBuses::kLcmUrlMemqNull) {
    // Note that there will be a set of params for each type of geometry.
    const std::vector<DrakeVisualizerParams> all_drake_params =
        internal::ConvertVisualizationConfigToDrakeParams(config);
    for (const DrakeVisualizerParams& params : all_drake_params) {
      DrakeVisualizer<double>::AddToBuilder(builder, *scene_graph, lcm, params);
    }
    if (config.publish_contacts) {
      ConnectContactResultsToDrakeVisualizer(builder, plant, *scene_graph, lcm,
                                             config.publish_period);
    }
  }

  // Add the Meshcat-based visualization, unless the user has opted out.
  if (meshcat == nullptr && config.enable_meshcat_creation) {
    meshcat = std::make_shared<geometry::Meshcat>();
  }
  if (meshcat != nullptr) {
    // Note that there will be a set of params for each type of geometry.
    const std::vector<MeshcatVisualizerParams> all_meshcat_params =
        internal::ConvertVisualizationConfigToMeshcatParams(config);
    for (const MeshcatVisualizerParams& params : all_meshcat_params) {
      MeshcatVisualizer<double>::AddToBuilder(builder, *scene_graph, meshcat,
                                              params);
    }
    if (config.publish_contacts) {
      ContactVisualizer<double>::AddToBuilder(
          builder, plant, meshcat,
          internal::ConvertVisualizationConfigToMeshcatContactParams(config));
    }
    if (config.publish_inertia && config.enable_alpha_sliders) {
      meshcat->SetSliderValue("inertia α", 0.5);
    }
  }

  // The inertia visualization layers atop SceneGraph, and therefore is not
  // specific to LCM vs Meshcat.
  if (config.publish_inertia) {
    InertiaVisualizer<double>::AddToBuilder(builder, plant, scene_graph);
  }
}

}  // namespace

void ApplyVisualizationConfig(const VisualizationConfig& config,
                              DiagramBuilder<double>* builder,
                              const LcmBuses* lcm_buses,
                              const MultibodyPlant<double>* plant,
                              SceneGraph<double>* scene_graph,
                              std::shared_ptr<geometry::Meshcat> meshcat,
                              DrakeLcmInterface* lcm) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  lcm = FindOrCreateLcmBus(lcm, lcm_buses, builder, "ApplyVisualizationConfig",
                           config.lcm_bus);
  DRAKE_DEMAND(lcm != nullptr);
  // N.B. The "a plant is required" precondition for ApplyVisualizationConfig
  // stems from the fact that we need to future-proof ourselves in case we
  // decide to add more kinds of visualization features by default, e.g., if
  // we decide to visualize properties of rigid bodies (e.g., their mass),
  // then we'll need the plant, not just the scene graph. Establishing the
  // precondition now means we won't break users down the road when we add that.
  if (plant == nullptr) {
    plant = &builder->GetDowncastSubsystemByName<MultibodyPlant>("plant");
  }
  if (scene_graph == nullptr) {
    scene_graph =
        &builder->GetMutableDowncastSubsystemByName<SceneGraph>("scene_graph");
  }
  ApplyVisualizationConfigImpl(config, lcm, meshcat, *plant, scene_graph,
                               builder);
}

void AddDefaultVisualization(DiagramBuilder<double>* builder,
                             std::shared_ptr<geometry::Meshcat> meshcat) {
  ApplyVisualizationConfig(VisualizationConfig{}, builder,
                           nullptr,  // lcm_buses
                           nullptr,  // plant
                           nullptr,  // scene_graph
                           meshcat);
}

namespace internal {

std::vector<DrakeVisualizerParams> ConvertVisualizationConfigToDrakeParams(
    const VisualizationConfig& config) {
  std::vector<DrakeVisualizerParams> result;

  if (config.publish_illustration) {
    DrakeVisualizerParams illustration;
    illustration.role = Role::kIllustration;
    illustration.publish_period = config.publish_period;
    illustration.default_color = config.default_illustration_color;
    result.push_back(illustration);
  }

  if (config.publish_proximity) {
    DrakeVisualizerParams proximity;
    proximity.role = Role::kProximity;
    proximity.publish_period = config.publish_period;
    proximity.default_color = config.default_proximity_color;
    proximity.show_hydroelastic = true;
    proximity.use_role_channel_suffix = true;
    result.push_back(proximity);
  }

  return result;
}

std::vector<MeshcatVisualizerParams> ConvertVisualizationConfigToMeshcatParams(
    const VisualizationConfig& config) {
  std::vector<MeshcatVisualizerParams> result;

  if (config.publish_illustration) {
    MeshcatVisualizerParams illustration;
    illustration.role = Role::kIllustration;
    illustration.publish_period = config.publish_period;
    illustration.default_color = config.default_illustration_color;
    illustration.prefix = std::string("illustration");
    illustration.delete_on_initialization_event =
        config.delete_on_initialization_event;
    illustration.enable_alpha_slider = config.enable_alpha_sliders;
    illustration.visible_by_default = true;
    result.push_back(illustration);
  }

  if (config.publish_inertia) {
    MeshcatVisualizerParams inertia;
    inertia.role = Role::kIllustration;
    inertia.publish_period = config.publish_period;
    inertia.prefix = std::string("inertia");
    inertia.delete_on_initialization_event =
        config.delete_on_initialization_event;
    inertia.enable_alpha_slider = config.enable_alpha_sliders;
    inertia.visible_by_default = false;
    inertia.include_unspecified_accepting = false;
    result.push_back(inertia);
  }

  if (config.publish_proximity) {
    MeshcatVisualizerParams proximity;
    proximity.role = Role::kProximity;
    proximity.publish_period = config.publish_period;
    proximity.default_color = config.default_proximity_color;
    proximity.prefix = std::string("proximity");
    proximity.delete_on_initialization_event =
        config.delete_on_initialization_event;
    proximity.enable_alpha_slider = config.enable_alpha_sliders;
    proximity.initial_alpha_slider_value = config.initial_proximity_alpha;
    proximity.visible_by_default = false;
    proximity.show_hydroelastic = true;
    result.push_back(proximity);
  }

  return result;
}

ContactVisualizerParams ConvertVisualizationConfigToMeshcatContactParams(
    const VisualizationConfig& config) {
  ContactVisualizerParams result;
  result.publish_period = config.publish_period;
  result.delete_on_initialization_event = config.delete_on_initialization_event;
  return result;
}

}  // namespace internal
}  // namespace visualization
}  // namespace drake
