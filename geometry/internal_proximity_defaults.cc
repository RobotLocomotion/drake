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

void BackfillDefaults(ProximityProperties* properties,
                      const SceneGraphConfig& config) {
  auto backfill = [&](const std::string& group_name, const std::string& name,
                      auto default_value) {
    if (properties->HasProperty(group_name, name)) { return; }
    if (!default_value.has_value()) { return; }
    properties->UpdateProperty(group_name, name, *default_value);
  };

  // Alias default_proximity_properties for convenience.
  const auto& props = config.default_proximity_properties;

  std::optional<HydroelasticType> wrapped_compliance;
  wrapped_compliance.emplace(internal::GetHydroelasticTypeFromString(
      props.compliance_type));
  backfill(kHydroGroup, kComplianceType, wrapped_compliance);

  backfill(kHydroGroup, kElastic, props.hydroelastic_modulus);
  backfill(kHydroGroup, kRezHint, props.resolution_hint);
  backfill(kHydroGroup, kSlabThickness, props.slab_thickness);

  backfill(kMaterialGroup, kHcDissipation, props.hunt_crossley_dissipation);
  if (props.static_friction.has_value()) {
    // DefaultProximityProperties::ValidateOrThrow() enforces invariants on
    // friction quantities.
    DRAKE_DEMAND(props.dynamic_friction.has_value());
    multibody::CoulombFriction<double> friction{
      *props.static_friction,
      *props.dynamic_friction,
    };
    std::optional<multibody::CoulombFriction<double>> wrapped_friction;
    wrapped_friction.emplace(friction);
    backfill(kMaterialGroup, kFriction, wrapped_friction);
  }
}

}  // namespace

template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const SceneGraphConfig& config) {
  DRAKE_DEMAND(geometry_state != nullptr);
  auto gids = geometry_state->GetAllGeometryIds(Role::kProximity);
  for (const auto& gid : gids) {
    ApplyProximityDefaults(geometry_state, config, gid);
  }
}

template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const SceneGraphConfig& config,
                            GeometryId geometry_id) {
  DRAKE_DEMAND(geometry_state != nullptr);
  auto gid = geometry_id;

  // TODO(#20820) Maybe this can be removed later.
  // Leave deformables untouched.
  if (geometry_state->IsDeformableGeometry(gid)) { return; }

  // Get current proximity properties -- some should exist, given the
  // filtering done above.
  auto found_props = geometry_state->GetProximityProperties(gid);
  DRAKE_DEMAND(found_props != nullptr);
  ProximityProperties props(*found_props);

  // Update proximity properties to provide defaults.
  BackfillDefaults(&props, config);

  // The source_id arguments on most scene graph methods are marked with todo's
  // for deprecation and removal. Until that happens, brute-force search for
  // the necessary source_id.
  SourceId gid_source_id;
  auto source_ids = geometry_state->GetAllSourceIds();
  for (const auto& source_id : source_ids) {
    if (geometry_state->BelongsToSource(gid, source_id)) {
      gid_source_id = source_id;
    }
  }

  // Make the final changes to proximity properties.
  geometry_state->AssignRole(gid_source_id, gid, props,
                             RoleAssign::kReplace);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<void(*)(GeometryState<T>*, const SceneGraphConfig&)>(
        &ApplyProximityDefaults<T>),
    static_cast<void(*)(GeometryState<T>*, const SceneGraphConfig&,
                        GeometryId)>(&ApplyProximityDefaults<T>)
))
}  // namespace internal
}  // namespace geometry
}  // namespace drake
