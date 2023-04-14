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

  // Note that there will be a set of params for each type of geometry.
  const std::vector<DrakeVisualizerParams> all_drake_params =
      internal::ConvertVisualizationConfigToDrakeParams(config);
  for (const DrakeVisualizerParams& params : all_drake_params) {
    // TODO(jwnimmer-tri) At the moment, meldis cannot yet display hydroelastic
    // geometry. So long as that's true, we should not enable it.
    DrakeVisualizerParams oopsie = params;
    oopsie.show_hydroelastic = false;
    DrakeVisualizer<double>::AddToBuilder(builder, *scene_graph, lcm, oopsie);
  }
  if (config.publish_contacts) {
    ConnectContactResultsToDrakeVisualizer(builder, plant, *scene_graph, lcm);
  }

  if (config.publish_inertia) {
    InertiaVisualizerParams inertia_params =
        internal::ConvertVisualizationConfigToInertiaParams(config);
    InertiaVisualizer<double>::AddToBuilder(builder, plant, scene_graph,
                                            inertia_params);
  }

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

// This is the deprecated overload.
void ApplyVisualizationConfig(const VisualizationConfig& config,
                              DiagramBuilder<double>* builder,
                              const LcmBuses* lcm_buses,
                              const MultibodyPlant<double>* plant,
                              const SceneGraph<double>* scene_graph,
                              std::shared_ptr<geometry::Meshcat> meshcat,
                              DrakeLcmInterface* lcm) {
  DRAKE_THROW_UNLESS(builder != nullptr);

  // Respell the const scene_graph pointer that the user gave us into a mutable
  // pointer instead. This is as simple as a const_cast, but first we need to
  // confirm that the const pointer was referring to something inside `builder`
  // to avoid any nasty surprises later on.
  SceneGraph<double>* mutable_scene_graph = nullptr;
  if (scene_graph != nullptr) {
    for (System<double>* system : builder->GetMutableSystems()) {
      DRAKE_DEMAND(system != nullptr);
      if (system == scene_graph) {
        mutable_scene_graph = const_cast<SceneGraph<double>*>(scene_graph);
        break;
      }
    }
    if (mutable_scene_graph == nullptr) {
      throw std::logic_error(
          "The const scene_graph provided to ApplyVisualizationConfig was not "
          "a System owned by the provided builder");
    }
  }

  // Delegate to the mutable overload.
  ApplyVisualizationConfig(config, builder, lcm_buses, plant,
                           mutable_scene_graph, std::move(meshcat), lcm);
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
    inertia.default_color = config.inertia_color;
    inertia.prefix = std::string("inertia");
    inertia.delete_on_initialization_event =
        config.delete_on_initialization_event;
    inertia.enable_alpha_slider = config.enable_alpha_sliders;
    inertia.alpha_slider_is_relative = false;
    inertia.visible_by_default = false;
    inertia.publish_untagged_geometry = false;
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

InertiaVisualizerParams ConvertVisualizationConfigToInertiaParams(
    const VisualizationConfig& config) {
  InertiaVisualizerParams result;
  result.color = config.inertia_color;
  // We force inertia geometry to be invisible to support visualizers
  // without alpha sliders. MeshcatVisualizer will reset the alpha of this
  // geometry based on the alpha slider.
  result.color.set(result.color.r(), result.color.g(), result.color.b(), 0.0);
  result.scale_factor = config.inertia_scale_factor;
  return result;
}

}  // namespace internal
}  // namespace visualization
}  // namespace drake
