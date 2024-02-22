#include "drake/geometry/internal_hydroelasticate.h"

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
  backfill(kHydroGroup, kRezHint, props.mesh_resolution_hint);
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

// Handle conversion details that require specific shape type
// information. Specifically, these are:
//  * implement `compliance_type_rigid_fallback` for non-convex surface meshes.
class ShapeAdjuster final : private ShapeReifier {
 public:
  // For meshes, modifies `props` for rigid fallback if it is both necessary
  // and requested in `config`.
  void MakeShapeAdjustments(const Shape& shape, const SceneGraphConfig& config,
                            ProximityProperties* props) {
    ReifyData data = {props, config};
    shape.Reify(this, &data);
  }

 private:
  using ShapeReifier::ImplementGeometry;

  struct ReifyData {
    ProximityProperties* props{};
    const SceneGraphConfig& config;
  };

  void AdjustMesh(ReifyData* data, const std::string& extension) {
    const auto& config = data->config;
    if (config.default_proximity_properties.compliance_type_rigid_fallback &&
        config.default_proximity_properties.compliance_type == "compliant" &&
        extension != ".vtk") {
      // Fall back to rigid geometry, if necessary and requested.
      data->props->UpdateProperty(kHydroGroup, kComplianceType,
                                  HydroelasticType::kRigid);
    }
  }

  void DefaultImplementGeometry(const Shape&) final {
    // Most shapes need no adjustment.
  }

  void ImplementGeometry(const Mesh& mesh, void* user_data) final {
    ReifyData* data = static_cast<ReifyData*>(user_data);
    AdjustMesh(data, mesh.extension());
  }
};

ShapeAdjuster* GetShapeAdjuster() {
  static never_destroyed<ShapeAdjuster> shape_adjuster;
  return &shape_adjuster.access();
}

}  // namespace

template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config) {
  DRAKE_DEMAND(geometry_state != nullptr);
  auto gids = geometry_state->GetAllGeometryIds(Role::kProximity);
  for (const auto& gid : gids) {
    Hydroelasticate(geometry_state, config, gid);
  }
}

template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                     const SceneGraphConfig& config, GeometryId geometry_id) {
  DRAKE_DEMAND(geometry_state != nullptr);
  auto gid = geometry_id;

  // Leave deformables untouched.
  if (geometry_state->IsDeformableGeometry(gid)) { return; }

  // Get current proximity properties -- some should exist, given the
  // filtering done above.
  auto found_props = geometry_state->GetProximityProperties(gid);
  DRAKE_DEMAND(found_props != nullptr);
  ProximityProperties props(*found_props);
  const HydroelasticType type = props.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);

  // Update proximity properties to provide defaults.
  BackfillDefaults(&props, config);

  // If the compliance type is defined, no adjustments are necessary.
  if (type != HydroelasticType::kUndefined) { return; }

  // Shape adjuster will fix configurations that can't work.
  GetShapeAdjuster()->MakeShapeAdjustments(
      geometry_state->GetShape(gid), config, &props);

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
        &Hydroelasticate<T>),
    static_cast<void(*)(GeometryState<T>*, const SceneGraphConfig&,
                        GeometryId)>(&Hydroelasticate<T>)
))
}  // namespace internal
}  // namespace geometry
}  // namespace drake
