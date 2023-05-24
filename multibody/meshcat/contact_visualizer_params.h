#pragma once

#include <string>

#include "drake/common/name_value.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace multibody {
namespace meshcat {

/** The set of parameters for configuring ContactVisualizer. */
struct ContactVisualizerParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(publish_period));
    a->Visit(DRAKE_NVP(color));
    a->Visit(DRAKE_NVP(hydro_force_color));
    a->Visit(DRAKE_NVP(hydro_moment_color));
    a->Visit(DRAKE_NVP(prefix));
    a->Visit(DRAKE_NVP(delete_on_initialization_event));
    a->Visit(DRAKE_NVP(force_threshold));
    a->Visit(DRAKE_NVP(moment_threshold));
    a->Visit(DRAKE_NVP(newtons_per_meter));
    a->Visit(DRAKE_NVP(newton_meters_per_meter));
    a->Visit(DRAKE_NVP(radius));
  }

  /** The duration (in simulation seconds) between attempts to update poses in
   the visualizer. (To help avoid small simulation time steps, we use a default
   period that has an exact representation in binary floating point; see
   drake#15021 for details.) */
  double publish_period{1 / 32.0};

  /** The color used to draw the point contact force arrows. */
  geometry::Rgba color{0, 1, 0, 1};

  /** The color used to draw the hydroelastic contact force arrows. */
  geometry::Rgba hydro_force_color{1, 0, 0, 1};

  /** The color used to draw the hydroelastic contact moment arrows. */
  geometry::Rgba hydro_moment_color{0, 0, 1, 1};

  /** A prefix to add to the path for all objects and transforms curated by the
   ContactVisualizer. It can be an absolute path or relative path.
   If relative, this `prefix` will be appended to the geometry::Meshcat `prefix`
   based on the standard path semantics in Meshcat. See @ref meshcat_path
   "Meshcat paths" for details. */
  std::string prefix{"contact_forces"};

  /** Determines whether to send a Meshcat::Delete(prefix) message on an
   initialization event to remove any visualizations e.g. from a previous
   simulation. See @ref declare_initialization_events "Declare initialization
   events" for more information. */
  bool delete_on_initialization_event{true};

  /** The threshold (in N) below which forces will no longer be drawn.
   The ContactVisualizer constructor enforces that this must be strictly
   positive; zero forces do not have a meaningful direction and cannot
   be visualized. */
  double force_threshold{0.01};

  /** The threshold (in N⋅m) below which moments will no longer be drawn.
   The ContactVisualizer constructor enforces that this must be strictly
   positive; zero moments do not have a meaningful direction and cannot
   be visualized. */
  double moment_threshold{0.01};

  /** Sets the length scale of the force vectors. */
  double newtons_per_meter{10};

  /** Sets the length scale of the moment vectors. */
  double newton_meters_per_meter{3};

  /** The radius of cylinder geometry used in the force/moment vector. */
  double radius{0.002};
};

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
