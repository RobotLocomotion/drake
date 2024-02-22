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
    a->Visit(DRAKE_NVP(enable_alpha_slider));
    a->Visit(DRAKE_NVP(initial_alpha_slider_value));
    a->Visit(DRAKE_NVP(visible_by_default));
    a->Visit(DRAKE_NVP(show_hydroelastic));
    a->Visit(DRAKE_NVP(include_unspecified_accepting));
  }

  /** The duration (in simulation seconds) between attempts to update poses in
   the visualizer. (To help avoid small simulation time steps, we use a default
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

  /** Determines whether to enable the alpha slider for geometry display. */
  bool enable_alpha_slider{false};

  /** Initial alpha slider value. This value should lie in the range [0, 1].
   Furthermore, the slider value is *quantized* which means the value used here
   will be replaced with the nearest quantized value supported by the slider
   implementation. */
  double initial_alpha_slider_value{1.0};

  /** Determines whether our meshcat path should be default to being visible. */
  bool visible_by_default{true};

  /** When using the hydroelastic contact model, collision geometries that are
   _declared_ as geometric primitives are frequently represented by some
   discretely tessellated mesh when computing contact. It can be quite helpful
   in assessing contact behavior to visualize these discrete meshes (in place
   of the idealized primitives).

   To visualize these representations it is necessary to request visualization
   of geometries with the Role::kProximity role (see the role field). It is
   further necessary to explicitly request the hydroelastic meshes where
   available (setting show_hydroelastic to `true`).

   Setting this `show_hydroelastic` to `true` will have no apparent effect if
   none of the collision meshes have a hydroelastic mesh associated with them.

   This option is ignored by MeshcatVisualizer<T> when T is not `double`, e.g.
   if T == AutoDiffXd. */
  bool show_hydroelastic{false};

  /** (Advanced) For a given geometry, if the GeometryProperties for our `role`
   has the property `(meshcat, accepting)` then the visualizer will show the
   geometry only if the property's value matches our `prefix`. If that property
   is absent then the geometry will be shown only if
   `include_unspecified_accepting` is true. */
  bool include_unspecified_accepting{true};
};

}  // namespace geometry
}  // namespace drake
