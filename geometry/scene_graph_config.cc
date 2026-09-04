#include "drake/geometry/scene_graph_config.h"

#include <stdexcept>
#include <string_view>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {

namespace {

// Check the value (if present) of `name`d `property` using `validate`. If the
// value is present and invalid, rethrow with a SceneGraphConfig-prefixed
// message that includes the property name.
void ThrowUnlessAbsentOr(std::string_view name, std::optional<double> property,
                         void (*validate)(double)) {
  if (!property.has_value()) {
    return;
  }
  try {
    validate(*property);
  } catch (const std::exception& e) {
    throw std::logic_error(fmt::format(
        "Invalid scene graph configuration: '{}' ({}) is invalid. {}", name,
        *property, e.what()));
  }
}

}  // namespace

void DefaultProximityProperties::ValidateOrThrow() const {
  // This will throw if the type is invalid.
  internal::GetHydroelasticTypeFromString(compliance_type);

// Use a macro to capture both property name and value.
#define DRAKE_ENFORCE(prop, validate) \
  ThrowUnlessAbsentOr(#prop, prop, &internal::validate)
  DRAKE_ENFORCE(hydroelastic_modulus, ThrowIfInvalidHydroelasticModulus);
  DRAKE_ENFORCE(resolution_hint, ThrowIfInvalidResolutionHint);
  DRAKE_ENFORCE(slab_thickness, ThrowIfInvalidSlabThickness);
  DRAKE_ENFORCE(margin, ThrowIfInvalidMargin);

  DRAKE_ENFORCE(dynamic_friction, ThrowIfInvalidFrictionCoefficient);
  DRAKE_ENFORCE(static_friction, ThrowIfInvalidFrictionCoefficient);
  DRAKE_ENFORCE(hunt_crossley_dissipation,
                ThrowIfInvalidHuntCrossleyDissipation);
  DRAKE_ENFORCE(relaxation_time, ThrowIfInvalidRelaxationTime);
  DRAKE_ENFORCE(point_stiffness, ThrowIfInvalidPointStiffness);
#undef DRAKE_ENFORCE

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
