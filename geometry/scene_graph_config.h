#pragma once

#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/// The set of configurable properties on a SceneGraph.
///
/// The field names and defaults here match SceneGraph's defaults exactly.
struct SceneGraphConfig {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(hydroelastize));
  }

  /// Configures the SceneGraph::set_hydroelastize().
  bool hydroelastize{false};

  // TODO(rpoyner-tri): add configurable values for use in hydroelastization.
};

}  // namespace geometry
}  // namespace drake
