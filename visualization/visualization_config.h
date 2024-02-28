#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/common/name_value.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace visualization {

/** Settings for what MultibodyPlant and SceneGraph should send to Meshcat
and/or Meldis.

See ApplyVisualizationConfig() for how to enact this configuration. */
struct VisualizationConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(lcm_bus));
    a->Visit(DRAKE_NVP(publish_period));
    a->Visit(DRAKE_NVP(publish_illustration));
    a->Visit(DRAKE_NVP(default_illustration_color));
    a->Visit(DRAKE_NVP(publish_proximity));
    a->Visit(DRAKE_NVP(default_proximity_color));
    a->Visit(DRAKE_NVP(initial_proximity_alpha));
    a->Visit(DRAKE_NVP(publish_contacts));
    a->Visit(DRAKE_NVP(publish_inertia));
    a->Visit(DRAKE_NVP(enable_meshcat_creation));
    a->Visit(DRAKE_NVP(delete_on_initialization_event));
    a->Visit(DRAKE_NVP(enable_alpha_sliders));
  }

  /** Which LCM URL to use.
  @see drake::systems::lcm::LcmBuses */
  std::string lcm_bus{"default"};

  /** The duration (in seconds) between published LCM messages that update
  visualization. (To help avoid small simulation time steps, we use a default
  period that has an exact representation in binary floating point;
  see drake#15021 for details.) */
  double publish_period{1 / 64.0};

  /** Whether to show illustration geometry. */
  bool publish_illustration{true};

  /** The color to apply to any illustration geometry that hasn't defined one.
  The vector must be of size three (rgb) or four (rgba). */
  geometry::Rgba default_illustration_color{0.9, 0.9, 0.9, 1.0};

  /** Whether to show proximity geometry. */
  bool publish_proximity{true};

  /** The color to apply to any proximity geometry that hasn't defined one.
  The vector must be of size three (rgb) or four (rgba). */
  geometry::Rgba default_proximity_color{0.8, 0, 0, 1.0};

  /** The initial value of the proximity alpha slider.
   Note: the effective transparency of the proximity geometry is the slider
   value multiplied by the alpha value of `default_proximity_color`. To have
   access to the full range of opacity, the color's alpha value should be one
   and the slider should be used to change it. */
  double initial_proximity_alpha{0.5};

  /** Whether to show body inertia. */
  bool publish_inertia{true};

  /** Whether to show contact forces. */
  bool publish_contacts{true};

  /** Whether to create a Meshcat object if needed. */
  bool enable_meshcat_creation{true};

  /** Determines whether to send a Meshcat::Delete() messages to the Meshcat
   object (if any) on an initialization event to remove any visualizations,
   e.g., from a previous simulation. */
  bool delete_on_initialization_event{true};

  /** Determines whether to enable alpha sliders for geometry display. */
  bool enable_alpha_sliders{false};
};

}  // namespace visualization
}  // namespace drake
