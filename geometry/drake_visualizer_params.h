#pragma once

#include "drake/common/name_value.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

/** The set of parameters for configuring DrakeVisualizer.  */
struct DrakeVisualizerParams {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(publish_period));
    a->Visit(DRAKE_NVP(role));
    a->Visit(DRAKE_NVP(default_color));
    a->Visit(DRAKE_NVP(show_hydroelastic));
    a->Visit(DRAKE_NVP(use_role_channel_suffix));
  }

  /** The duration (in seconds) between published LCM messages that update the
   poses of the scene's geometry. (To help avoid small simulation time steps, we
   use a default period that has an exact representation in binary floating
   point; see drake#15021 for details.) */
  double publish_period{1 / 64.0};

  /** The role of the geometries to be sent to the visualizer.  */
  Role role{Role::kIllustration};

  /** The color to apply to any geometry that hasn't defined one.  */
  Rgba default_color{0.9, 0.9, 0.9, 1.0};

  /** When using the hydroelastic contact model, collision geometries that are
   _declared_ as geometric primitives are frequently represented by some
   discretely tessellated mesh when computing contact. It can be quite helpful
   in assessing contact behavior to visualize these discrete meshes (in place of
   the idealized primitives).

   To visualize these representations it is necessary to request visualization
   of geometries with the Role::kProximity role (see the role field). It is
   further necessary to explicitly request the hydroelastic meshes where
   available (setting show_hydroelastic to `true`).

   Setting this `show_hydroelastic` to `true` will have no apparent effect if
   none of the collision meshes have a hydroelastic mesh associated with them.
   */
  bool show_hydroelastic{false};

  /** Setting this to `true` will cause LCM channel names to have a suffix
   appended, allowing simultaneous transmission of multiple geometry roles via
   multiple DrakeVisualizer instances. See DrakeVisualizer for details. */
  bool use_role_channel_suffix{false};
};

}  // namespace geometry
}  // namespace drake
