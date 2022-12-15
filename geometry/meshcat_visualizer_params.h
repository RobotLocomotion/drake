#pragma once

#include <string>

#include "drake/common/name_value.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

/** The set of parameters for configuring MeshcatVisualizer. */
struct MeshcatVisualizerParams {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(publish_period));
    a->Visit(DRAKE_NVP(role));
    a->Visit(DRAKE_NVP(default_color));
    a->Visit(DRAKE_NVP(prefix));
    a->Visit(DRAKE_NVP(delete_on_initialization_event));
    a->Visit(DRAKE_NVP(enable_alpha_sliders));
  }

  /** The duration (in simulation seconds) between attempts to update poses in
   the visualizer. (To help avoid small simulation timesteps, we use a default
   period that has an exact representation in binary floating point; see
   drake#15021 for details.) */
  double publish_period{1 / 64.0};

  /** The role of the geometries to be sent to the visualizer. */
  Role role{Role::kIllustration};

  /** The color to apply to any geometry that hasn't defined one. */
  Rgba default_color{0.9, 0.9, 0.9, 1.0};

  /** A prefix to add to the path for all objects and transforms curated by the
   MeshcatVisualizer. It can be an absolute path or relative path. If relative,
   this `prefix` will be appended to the Meshcat `prefix` based on the standard
   path semantics in Meshcat.  See @ref meshcat_path "Meshcat paths" for
   details. */
  std::string prefix{"visualizer"};

  /** Determines whether to send a Meshcat::Delete(prefix) message on an
   initialization event to remove any visualizations e.g. from a previous
   simulation. See @ref declare_initialization_events "Declare initialization
   events" for more information. */
  bool delete_on_initialization_event{true};

  /** Determines whether to enable alpha sliders for geometry display. */
  bool enable_alpha_sliders{false};
};

}  // namespace geometry
}  // namespace drake
