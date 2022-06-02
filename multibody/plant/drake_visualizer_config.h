#pragma once

#include "drake/geometry/drake_visualizer_params.h"

namespace drake {
namespace multibody {

/// The set of configurable properties for (potentially multiple)
/// DrakeVisualizer systems in a multibody diagram.
///
// TODO(rpoyner-tri): serialization?
struct DrakeVisualizerConfig {
  /// Geometry roles.
  geometry::DrakeVisualizerMultiRoleParams geometry_config;

  /// Contact results.
  // Note: Use the publish period already within geometry_config.
  bool show_contact_results{true};
};

}  // namespace multibody
}  // namespace drake
