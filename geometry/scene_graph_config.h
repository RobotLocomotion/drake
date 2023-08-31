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
    a->Visit(DRAKE_NVP(hydroelastize_default_hydroelastic_modulus));
    a->Visit(DRAKE_NVP(hydroelastize_default_resolution_hint));
    a->Visit(DRAKE_NVP(hydroelastize_default_slab_thickness));
  }

  /// Configures the SceneGraph::set_hydroelastize().
  bool hydroelastize{false};

  // TODO(rpoyner-tri): Choose reasonable and sane default values; these are
  // just guesses.

  /// Configures the default hydroelastic modulus to use when hydroelastize is
  /// active.
  double hydroelastize_default_hydroelastic_modulus{1e8};
  /// Configures the default resolution hint to use when hydroelastize is
  /// active.
  double hydroelastize_default_resolution_hint{0.01};
  /// Configures the default slab thickness to use when hydroelastize is
  /// active.
  double hydroelastize_default_slab_thickness{1e3};
};

}  // namespace geometry
}  // namespace drake
