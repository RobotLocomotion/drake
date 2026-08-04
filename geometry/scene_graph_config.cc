#include "drake/geometry/scene_graph_config.h"

#include <functional>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {

namespace {

// Conditions, that if not met, could trigger an exception.
// TODO(#21167) NaN values are not consistently accounted for.
enum Condition {
  kPositiveFinite,
  kNonNegativeFinite,
  kPositive,
  kNonNegative,
};

// Check the value (if present) of `name`d `property` for `condition`. If the
// value is present and the condition is not met, throw an exception with a
// nice message.
void ThrowUnlessAbsentOr(std::string_view name, std::optional<double> property,
                         Condition condition) {
  if (!property.has_value()) {
    return;
  }
  double value = *property;
  std::string_view condition_name;
  bool should_throw{false};
  switch (condition) {
    case kPositiveFinite: {
      should_throw = (!std::isfinite(value) || value <= 0.0);
      condition_name = "positive, finite";
      break;
    }
    case kNonNegativeFinite: {
      should_throw = (!std::isfinite(value) || value < 0.0);
      condition_name = "non-negative, finite";
      break;
    }
    case kPositive: {
      should_throw = (value <= 0.0);
      condition_name = "positive";
      break;
    }
    case kNonNegative: {
      should_throw = (value < 0.0);
      condition_name = "non-negative";
      break;
    }
  }
  if (should_throw) {
    throw std::logic_error(fmt::format(
        "Invalid scene graph configuration: '{}' ({}) must be a {} value.",
        name, value, condition_name));
  }
}

}  // namespace

void DefaultProximityProperties::ValidateOrThrow() const {
  // This will throw if the type is invalid.
  internal::GetHydroelasticTypeFromString(compliance_type);

// Use a macro to capture both property name and value.
#define DRAKE_ENFORCE(prop, cond) ThrowUnlessAbsentOr(#prop, prop, cond)
  DRAKE_ENFORCE(hydroelastic_modulus, kPositive);
  DRAKE_ENFORCE(resolution_hint, kPositiveFinite);
  DRAKE_ENFORCE(slab_thickness, kPositiveFinite);
  DRAKE_ENFORCE(margin, kNonNegativeFinite);

  DRAKE_ENFORCE(dynamic_friction, kNonNegative);
  DRAKE_ENFORCE(static_friction, kNonNegative);
  DRAKE_ENFORCE(hunt_crossley_dissipation, kNonNegative);
  DRAKE_ENFORCE(relaxation_time, kNonNegativeFinite);
  DRAKE_ENFORCE(point_stiffness, kPositive);
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
