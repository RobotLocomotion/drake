#include "drake/geometry/scene_graph_config.h"

#include <functional>

#include "drake/common/drake_throw.h"
#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {

// TODO(rpoyner-tri): re-write this for better, more friendly error messages.

void ThrowUnlessAbsentOr(
    std::optional<double> property,
    std::function<bool(double)> predicate) {
  if (property.has_value()) {
    DRAKE_THROW_UNLESS(predicate(*property));
  }
}

void DefaultProximityProperties::ValidateOrThrow() const {
  // This will throw if the type is invalid.
  internal::GetHydroelasticTypeFromString(compliance_type);

  auto isfinite = [](double x) { return std::isfinite(x); };
  ThrowUnlessAbsentOr(hydroelastic_modulus, isfinite);
  ThrowUnlessAbsentOr(mesh_resolution_hint, isfinite);
  ThrowUnlessAbsentOr(slab_thickness, isfinite);
  ThrowUnlessAbsentOr(dynamic_friction, isfinite);
  ThrowUnlessAbsentOr(static_friction, isfinite);
  ThrowUnlessAbsentOr(hunt_crossley_dissipation, isfinite);
  ThrowUnlessAbsentOr(relaxation_time, isfinite);
  ThrowUnlessAbsentOr(point_stiffness, isfinite);

  auto positive = [](double x) { return x > 0.0; };
  auto nonnegative = [](double x) { return x >= 0.0; };
  ThrowUnlessAbsentOr(hydroelastic_modulus, positive);
  ThrowUnlessAbsentOr(mesh_resolution_hint, positive);
  ThrowUnlessAbsentOr(slab_thickness, positive);
  ThrowUnlessAbsentOr(dynamic_friction, nonnegative);
  ThrowUnlessAbsentOr(static_friction, nonnegative);
  ThrowUnlessAbsentOr(hunt_crossley_dissipation, nonnegative);
  ThrowUnlessAbsentOr(relaxation_time, nonnegative);
  ThrowUnlessAbsentOr(point_stiffness, nonnegative);

  // Require either both friction quantities or neither.
  DRAKE_THROW_UNLESS(static_friction.has_value() ==
                     dynamic_friction.has_value());
  if (static_friction.has_value()) {
    // Since we can't conveniently use multibody::CoulombFriction, check now
    // that the values are compatible.
    DRAKE_THROW_UNLESS(*static_friction >= *dynamic_friction);
  }
}

void SceneGraphConfig::ValidateOrThrow() const {
  default_proximity_properties.ValidateOrThrow();
}

}  // namespace geometry
}  // namespace drake

