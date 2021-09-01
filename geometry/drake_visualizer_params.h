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

   Because of communication mechanism between DrakeVisualizer and
   `drake_visualizer`, it is necessary for DrakeVisualizer to write out an
   OBJ-formatted file representing each hydroelastic mesh. DrakeVisualizer will
   create a temporary directory and write OBJ files to that directory, cleaning
   it up upon destruction. %DrakeVisualizer uses the temp_directory() function
   to create the temporary, see that function's documentation for how to
   influence the location of the temporary directory. */
  bool show_hydroelastic{false};

  // TODO(SeanCurtis-TRI): Consider adding a flag asking that the directory
  //  and its contents *not* be cleaned up so that the user can examine the
  //  generated OBJs after the fact.

  // TODO(SeanCurtis-TRI): Consider allowing the user to specify a directory.
  //  What would the "clean up" semantics be? Delete the files this session
  //  created? Delete the directory? Etc. Determine if that has value.
};

}  // namespace geometry
}  // namespace drake
