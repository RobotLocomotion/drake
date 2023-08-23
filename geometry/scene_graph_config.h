#pragma once

#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/// Configurable properties for the automatic hydroelasticate feature.
///
/// Controls whether the hydroelasticate feature is enabled, and what default
/// values are supplied for proximity properties to geometries that are not
/// annotated for hydroelastic contact in the scene graph model.
///
/// @see @ref hug_hydroelasticate, @ref hug_properties,
/// @ref stribeck_approximation.
struct HydroelasticateConfig {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(enabled));
    a->Visit(DRAKE_NVP(minimum_primitive_size));
    a->Visit(DRAKE_NVP(default_hydroelastic_modulus));
    a->Visit(DRAKE_NVP(default_mesh_resolution_hint));
    a->Visit(DRAKE_NVP(default_slab_thickness));
    a->Visit(DRAKE_NVP(default_hunt_crossley_dissipation));
    a->Visit(DRAKE_NVP(default_dynamic_friction));
    a->Visit(DRAKE_NVP(default_static_friction));
  }
  /// If true, the hydroelasticate feature will be applied to scene graph model
  /// data when new contexts are created.
  bool enabled{false};
  /// Configures the minimum primitive geometry size (in meters) to consider
  /// for hydroelastic contact. Primitives smaller than this size will have
  /// their proximity role removed. Mesh geometries are not affected by this
  /// setting.
  double minimum_primitive_size{1e-4};
  /// Configures the default hydroelastic modulus (in pascals) to use when
  /// hydroelasticate is active.
  double default_hydroelastic_modulus{1e7};
  /// Configures the default resolution hint (in meters) to use when
  /// hydroelasticate is active.
  double default_mesh_resolution_hint{0.01};
  /// Configures the default slab thickness (in meters) to use when
  /// hydroelasticate is active. Only has an effect on half space geometry
  /// elements.
  double default_slab_thickness{10.0};
  /// Configures the default dissipation (in seconds per meter) to use when
  /// hydroelasticate is active.
  double default_hunt_crossley_dissipation{1.25};
  /// Configures the default dynamic friction (dimensionless) to use when
  /// hydroelasticate is active.
  double default_dynamic_friction{0.5};
  /// Configures the default dynamic friction (dimensionless) to use when
  /// hydroelasticate is active.
  double default_static_friction{0.5};
};

/// The set of configurable properties on a SceneGraph.
///
/// The field names and defaults here match SceneGraph's defaults exactly.
struct SceneGraphConfig {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(hydroelasticate));
  }

  /// Configures the SceneGraph hydroelasticate feature.
  HydroelasticateConfig hydroelasticate{};
};

}  // namespace geometry
}  // namespace drake
