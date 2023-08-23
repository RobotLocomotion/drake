#pragma once

#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/// Configurable properties for the automatic hydroelastic feature.
///
/// Controls whether the hydroelastic feature is enabled, and what default
/// values are supplied for proximity properties to geometries that are not
/// annotated for hydroelastic contact in the scene graph model.
///
/// @see @ref hug_hydroelastic, @ref hug_properties,
/// @ref stribeck_approximation.
struct HydroelasticConfig {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(enabled));
    a->Visit(DRAKE_NVP(minimum_primitive_size));
    a->Visit(DRAKE_NVP(default_hydroelastic_modulus));
    a->Visit(DRAKE_NVP(default_mesh_resolution_hint));
    a->Visit(DRAKE_NVP(default_slab_thickness));
  }
  /// If true, the hydroelastic feature will be applied to scene graph model
  /// data when new contexts are created.
  bool enabled{false};
  /// Configures the minimum primitive geometry size (in meters) to consider
  /// for hydroelastic contact. Primitives smaller than this size will have
  /// their proximity role removed. Mesh geometries are not affected by this
  /// setting.
  double minimum_primitive_size{1e-4};
  /// Configures the default hydroelastic modulus (in pascals) to use when
  /// hydroelastic is active.
  double default_hydroelastic_modulus{1e7};
  /// Configures the default resolution hint (in meters) to use when
  /// hydroelastic is active.
  double default_mesh_resolution_hint{0.01};
  /// Configures the default slab thickness (in meters) to use when
  /// hydroelastic is active. Only has an effect on half space geometry
  /// elements.
  double default_slab_thickness{10.0};
};

/// The set of configurable properties on a SceneGraph.
///
/// The field names and defaults here match SceneGraph's defaults exactly.
struct SceneGraphConfig {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(hydroelastic));
  }

  /// Configures the SceneGraph hydroelastic feature.
  HydroelasticConfig hydroelastic{};
};

}  // namespace geometry
}  // namespace drake
