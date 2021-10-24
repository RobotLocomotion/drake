#pragma once

#include <string>

#include "drake/geometry/rgba.h"

namespace drake {
namespace multibody {

/** The set of parameters for configuring ContactResultsToMeshcat. */
struct ContactResultsToMeshcatParams {
  /** The duration (in simulation seconds) between attempts to update poses in
   the visualizer. (To help avoid small simulation timesteps, we use a default
   period that has an exact representation in binary floating point; see
   drake#15021 for details.) */
  double publish_period{1 / 32.0};

  /** The color used to draw the contact force arrows. */
  geometry::Rgba color{0, 1, 0, 1};

  /** A prefix to add to the path for all objects and transforms curated by the
   ContactResultsToMeshcat. It can be an absolute path or relative path.
   If relative, this `prefix` will be appended to the geometry::Meshcat `prefix`
   based on the standard path semantics in Meshcat. See @ref meshcat_path
   "Meshcat paths" for details. */
  std::string prefix{"contact_forces"};

  /** Determines whether to send a Meschat::Delete(prefix) message on an
   initialization event to remove any visualizations e.g. from a previous
   simulation. See @ref declare_initialization_events "Declare initialization
   events" for more information. */
  bool delete_on_initialization_event{true};

  /** The threshold below which forces will no longer be drawn.  The
   ContactResultsToMeshcat constructor enforces that this must be strictly
   positive; zero forces do not have a meaningful direction and cannot be
   visualized. */
  double force_threshold{0.01};

  /** Sets the length scale of the force vectors. */
  double newtons_per_meter{10};

  /** The radius of cylinder geometry used in the force vector . */
  double radius{0.01};
};

}  // namespace multibody
}  // namespace drake
