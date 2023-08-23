#pragma once

#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/// @see @ref hug_hydroelastic, @ref hug_properties,
/// @ref stribeck_approximation.
struct DefaultProximityProperties {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(compliance_type));
    a->Visit(DRAKE_NVP(compliance_type_rigid_fallback));
    a->Visit(DRAKE_NVP(hydroelastic_modulus));
    a->Visit(DRAKE_NVP(mesh_resolution_hint));
    a->Visit(DRAKE_NVP(slab_thickness));
    a->Visit(DRAKE_NVP(hunt_crossley_dissipation));
    a->Visit(DRAKE_NVP(dynamic_friction));
    a->Visit(DRAKE_NVP(static_friction));
  }
  /** There are three valid options for `compliance_type`:
  - "undefined": hydroelastic contact will not be used.
  - "rigid": the default hydroelastic compliance type will be rigid;
  note that rigid-rigid contact is not supported by the hydroelastic
  contact model, but is supported by point contact.
  - "compliant": the default hydroelastic compliance type will be compliant;
  note that `compliance_type_rigid_fallback` offers a caveat for shapes
  that do not support compliant contact.
  */
  std::string compliance_type{"undefined"};
  /** If default compliance_type is "compliant" but the shape does not support
  compliant contact (currently only for non-convex meshes) then the compliance
  type will fall back to "rigid". */
  bool compliance_type_rigid_fallback{true};
  double hydroelastic_modulus{1e7};
  double mesh_resolution_hint{0.01};
  double slab_thickness{10.0};
  double hunt_crossley_dissipation{1.25};
  double dynamic_friction{0.5};
  double static_friction{0.5};
};

/// The set of configurable properties on a SceneGraph.
///
/// The field names and defaults here match SceneGraph's defaults exactly.
struct SceneGraphConfig {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_proximity_properties));
  }

  /// Configures the SceneGraph hydroelastic feature.
  DefaultProximityProperties default_proximity_properties;
};

}  // namespace geometry
}  // namespace drake
