#include "drake/geometry/internal_proximity_defaults.h"

#include <algorithm>
#include <optional>
#include <string>

#include "drake/common/never_destroyed.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

// @return true if any properties were modified.
bool BackfillDefaults(ProximityProperties* properties,
                      const DefaultProximityProperties& defaults) {
  auto backfill = [&](const std::string& group_name, const std::string& name,
                      const auto& default_value) -> bool {
    if (properties->HasProperty(group_name, name)) {
      return false;
    }
    if (!default_value.has_value()) {
      return false;
    }
    properties->UpdateProperty(group_name, name, *default_value);
    return true;
  };

  bool result = false;
  std::optional<HydroelasticType> wrapped_compliance(
      internal::GetHydroelasticTypeFromString(defaults.compliance_type));
  result |= backfill(kHydroGroup, kComplianceType, wrapped_compliance);

  result |= backfill(kHydroGroup, kElastic, defaults.hydroelastic_modulus);
  result |= backfill(kHydroGroup, kRezHint, defaults.resolution_hint);
  result |= backfill(kHydroGroup, kSlabThickness, defaults.slab_thickness);

  result |= backfill(kMaterialGroup, kHcDissipation,
                     defaults.hunt_crossley_dissipation);
  if (defaults.static_friction.has_value()) {
    // DefaultProximityProperties::ValidateOrThrow() enforces invariants on
    // friction quantities.
    DRAKE_DEMAND(defaults.dynamic_friction.has_value());
    const auto wrapped_friction =
        std::make_optional<multibody::CoulombFriction<double>>(
            *defaults.static_friction, *defaults.dynamic_friction);
    result |= backfill(kMaterialGroup, kFriction, wrapped_friction);
  }
  return result;
}

}  // namespace

template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const DefaultProximityProperties& defaults) {
  DRAKE_DEMAND(geometry_state != nullptr);
  for (const auto& geometry_id :
       geometry_state->GetAllGeometryIds(Role::kProximity)) {
    ApplyProximityDefaults(geometry_state, defaults, geometry_id);
  }
}

template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const DefaultProximityProperties& defaults,
                            GeometryId geometry_id) {
  DRAKE_DEMAND(geometry_state != nullptr);

  // TODO(#20820) Maybe this can be removed later.
  // Leave deformables untouched.
  if (geometry_state->IsDeformableGeometry(geometry_id)) {
    return;
  }

  // Get current proximity properties -- some should exist, given the
  // filtering done above.
  const auto* found_props = geometry_state->GetProximityProperties(geometry_id);
  DRAKE_DEMAND(found_props != nullptr);
  ProximityProperties props(*found_props);

  // Update proximity properties to provide defaults. Return early if nothing
  // changed.
  bool changed = BackfillDefaults(&props, defaults);
  if (!changed) {
    return;
  }

  // Make the final changes to proximity properties.
  geometry_state->AssignRole(geometry_state->get_source_id(geometry_id),
                             geometry_id, props, RoleAssign::kReplace);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<void (*)(GeometryState<T>*, const DefaultProximityProperties&)>(
        &ApplyProximityDefaults<T>),
    static_cast<void (*)(GeometryState<T>*, const DefaultProximityProperties&,
                         GeometryId)>(&ApplyProximityDefaults<T>)))
}  // namespace internal
}  // namespace geometry
}  // namespace drake
