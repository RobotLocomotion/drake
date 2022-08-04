#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/common/name_value.h"

namespace drake {
namespace visualization {

// TODO(jwnimmer-tri) Add an option run a Meshcat server within the builder,
// and/or disable LCM entirely.

/** Settings for what MultibodyPlant and SceneGraph should send to meldis
and/or drake_visualizer.

@experimental The exact configuration details (names and types) are subject to
change as we polish this new feature.

See ApplyVisualizerConfig() for how to enact this configuration. */
struct VisualizerConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(lcm_bus));
    a->Visit(DRAKE_NVP(publish_period));
    a->Visit(DRAKE_NVP(publish_illustration));
    a->Visit(DRAKE_NVP(default_illustration_color_rgba));
    a->Visit(DRAKE_NVP(publish_proximity));
    a->Visit(DRAKE_NVP(default_proximity_color_rgba));
    a->Visit(DRAKE_NVP(publish_contacts));
  }

  /** Which LCM URL to use. */
  std::string lcm_bus{"default"};

  /** The duration (in seconds) between published LCM messages that update
  visualization. (To help avoid small simulation timesteps, we use a default
  period that has an exact representation in binary floating point;
  see drake#15021 for details.) */
  double publish_period{1 / 64.0};

  /** Whether to show illustration geometry. */
  bool publish_illustration{true};

  /** The color to apply to any illustration geometry that hasn't defined one.
  The vector must be of size three (rgb) or four (rgba). */
  Eigen::VectorXd default_illustration_color_rgba{
      Eigen::Vector3d{0.9, 0.9, 0.9}};

  /** Whether to show proximity geometry. */
  bool publish_proximity{true};

  /** The color to apply to any proximity geometry that hasn't defined one.
  The vector must be of size three (rgb) or four (rgba). */
  Eigen::VectorXd default_proximity_color_rgba{
      Eigen::Vector4d{1, 0, 0, 0.5}};

  /** Whether to show contact forces. */
  bool publish_contacts{true};
};

}  // namespace visualization
}  // namespace drake
