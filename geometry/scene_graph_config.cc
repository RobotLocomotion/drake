#include "drake/geometry/scene_graph_config.h"

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {

void DefaultProximityProperties::ValidateOrThrow() const {
  // This will throw if the type is invalid.
  internal::GetHydroelasticTypeFromString(compliance_type);

  if (hydroelastic_modulus.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsPositive(*hydroelastic_modulus),
                       *hydroelastic_modulus);
  }
  if (resolution_hint.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsPositiveFinite(*resolution_hint),
                       *resolution_hint);
  }
  if (slab_thickness.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsPositiveFinite(*slab_thickness),
                       *slab_thickness);
  }
  if (margin.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsNonNegativeFinite(*margin), *margin);
  }

  if (dynamic_friction.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsNonNegative(*dynamic_friction),
                       *dynamic_friction);
  }
  if (static_friction.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsNonNegative(*static_friction),
                       *static_friction);
  }
  if (hunt_crossley_dissipation.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsNonNegative(*hunt_crossley_dissipation),
                       *hunt_crossley_dissipation);
  }
  if (relaxation_time.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsNonNegativeFinite(*relaxation_time),
                       *relaxation_time);
  }
  if (point_stiffness.has_value()) {
    DRAKE_THROW_UNLESS(internal::IsPositive(*point_stiffness),
                       *point_stiffness);
  }

  // Require either both friction quantities or neither.
  if (static_friction.has_value() != dynamic_friction.has_value()) {
    auto value_or_nullopt = [](auto x) {
      return x ? fmt::to_string(*x) : "nullopt";
    };
    throw std::logic_error(fmt::format(
        "Invalid scene graph configuration: either both 'static_friction' ({})"
        " and 'dynamic_friction' ({}) must have a value, or neither.",
        value_or_nullopt(static_friction), value_or_nullopt(dynamic_friction)));
  }
  if (static_friction.has_value()) {
    // The constructor throws nice messages if its invariants fail.
    multibody::CoulombFriction coulomb{*static_friction, *dynamic_friction};
  }
}

void SceneGraphConfig::ValidateOrThrow() const {
  default_proximity_properties.ValidateOrThrow();
}

}  // namespace geometry
}  // namespace drake
