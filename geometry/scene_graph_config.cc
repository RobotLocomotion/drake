#include "drake/geometry/scene_graph_config.h"

#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {

namespace {

void ThrowIfPresentAndInvalid(
    const std::optional<double>& property,
    const std::function<std::optional<std::string>(double)>& report) {
  if (property.has_value()) {
    if (auto error = report(*property)) {
      throw std::logic_error(*error);
    }
  }
}

}  // namespace

void DefaultProximityProperties::ValidateOrThrow() const {
  // This will throw if the type is invalid.
  internal::GetHydroelasticTypeFromString(compliance_type);

  ThrowIfPresentAndInvalid(hydroelastic_modulus,
                           internal::ReportIfInvalidHydroelasticModulus);
  ThrowIfPresentAndInvalid(resolution_hint,
                           internal::ReportIfInvalidResolutionHint);
  ThrowIfPresentAndInvalid(slab_thickness,
                           internal::ReportIfInvalidSlabThickness);
  ThrowIfPresentAndInvalid(margin, internal::ReportIfInvalidMargin);

  ThrowIfPresentAndInvalid(dynamic_friction,
                           internal::ReportIfInvalidFrictionCoefficient);
  ThrowIfPresentAndInvalid(static_friction,
                           internal::ReportIfInvalidFrictionCoefficient);
  ThrowIfPresentAndInvalid(hunt_crossley_dissipation,
                           internal::ReportIfInvalidHuntCrossleyDissipation);
  ThrowIfPresentAndInvalid(relaxation_time,
                           internal::ReportIfInvalidRelaxationTime);
  ThrowIfPresentAndInvalid(point_stiffness,
                           internal::ReportIfInvalidPointStiffness);

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
