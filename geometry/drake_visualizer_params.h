#pragma once

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

/** The set of parameters for configuring DrakeVisualizer.  */
struct DrakeVisualizerParams {
  /** The duration (in seconds) between published LCM messages that update the
   poses of the scene's geometry.  */
  double publish_period{1 / 60.0};

  /** The role of the geometries to be sent to the visualizer.  */
  Role role{Role::kIllustration};

  /** The color to apply to any geometry that hasn't defined one.  */
  Rgba default_color{0.9, 0.9, 0.9, 1.0};
};

}  // namespace geometry
}  // namespace drake
