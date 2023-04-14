#pragma once

#include "drake/common/name_value.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace visualization {

/** The set of parameters for configuring InertiaVisualizer. */
struct InertiaVisualizerParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(color));
  }

  /** The color to apply to inertia visualization geometry.
  This is the same color used by VisualizationConfig, but the
  visualization config functions set the alpha to zero and allow
  the associated alpha slider to make the geometry visible. */
  geometry::Rgba color{0.0, 0.0, 1.0, 0.2};
};

}  // namespace visualization
}  // namespace drake
