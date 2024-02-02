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

  std::optional<HydroelasticType> optional_compliance{
    internal::GetHydroelasticTypeFromString(
        config.default_proximity_properties.compliance_type)
  };
  backfill(kHydroGroup, kComplianceType, optional_compliance);
  backfill(kHydroGroup, kElastic,
           config.default_proximity_properties.hydroelastic_modulus);
  backfill(kHydroGroup, kRezHint,
           config.default_proximity_properties.mesh_resolution_hint);
  backfill(kHydroGroup, kSlabThickness,
           config.default_proximity_properties.slab_thickness);

  backfill(kMaterialGroup, kHcDissipation,
           config.default_proximity_properties.hunt_crossley_dissipation);
  if (config.default_proximity_properties.static_friction.has_value()) {
    multibody::CoulombFriction<double> friction{
      *config.default_proximity_properties.static_friction,
      *config.default_proximity_properties.dynamic_friction,
    };
    std::optional<multibody::CoulombFriction<double>> optional_friction{
      friction
    };
    backfill(kMaterialGroup, kFriction, optional_friction);
  }
}

// Handle conversion details that require specific shape type
// information. Specifically, these are:
//  * implement `compliance_type_rigid_fallback` for non-convex surface meshes.
//  * recommend too-small primitive geometries for later removal.
class ShapeAdjuster final : private ShapeReifier {
 public:
  // For meshes, modifies `props` for rigid fallback if it is both necessary
  // and requested in `config`. For primitives, detect if the geometry is too
  // small.
  //
  // @returns true if the shape is too small to participate in
  // hydroelastic contact for this scene.
  bool MakeShapeAdjustments(const Shape& shape, const SceneGraphConfig& config,
                            ProximityProperties* props) {
    ReifyData data = {props, config, false};
    shape.Reify(this, &data);
    return data.is_too_small;
  }

 private:
  using ShapeReifier::ImplementGeometry;

  struct ReifyData {
    ProximityProperties* props{};
    const SceneGraphConfig& config;
    bool is_too_small{false};
  };

  // If a the `max_dimension` is less than some threshold, flag the
  // shape as too small in the `data` struct. `max_dimension` is
  // defined as 0.5 * max(bx, by, bz) where (bx, by, bz) are the
  // dimensions of the smallest AABB (in the shape's cannonical frame)
  // that contains the shape.
  //
  // Note that this heuristic does not flag shapes that are microscopic in
  // only some dimensions: flat sheets, or long skinny shapes. Those may
  // still hit some geometry exception (near-zero tetrahedron volume was the
  // important one at the time of this writing).
  void CheckTooSmall(ReifyData* data, double max_dimension) {
    // Congratulations. Your geometry is smaller than a speck of household
    // dust. For various numerical reasons, we restrict the size of primitives
    // to those that are 'large' enough by our `max_dimension` heuristic.
    const double kTooSmall = 2e-5;

    if (max_dimension < kTooSmall) {
      data->is_too_small = true;
    }
  }

  void AdjustMesh(ReifyData* data, const std::string& extension) {
    // For now, let us claim that anything supplied as a mesh will likely be
    // appropriately sized to be part of the scene, and not a point-contact
    // helper shape. Therefore, too-small checking should not be
    // necessary. TODO(rpoyner-tri): Revisit this if there are
    // counter-examples.

    const auto& config = data->config;
    if (config.default_proximity_properties.compliance_type_rigid_fallback &&
        config.default_proximity_properties.compliance_type == "compliant" &&
        extension != ".vtk") {
      // Fall back to rigid geometry, if necessary and requested.
      data->props->UpdateProperty(kHydroGroup, kComplianceType,
                                  HydroelasticType::kRigid);
    }
  }

  void ImplementGeometry(const Box& box, void* user_data) final {
    ReifyData* data = static_cast<ReifyData*>(user_data);
    double max_dimension =
        0.5 * std::max(box.width(), std::max(box.depth(), box.height()));
    CheckTooSmall(data, max_dimension);
  }

  void ImplementGeometry(const Capsule& capsule, void* user_data) final {
    ReifyData* data = static_cast<ReifyData*>(user_data);
    double max_dimension = capsule.length() * 0.5 + capsule.radius();
    CheckTooSmall(data, max_dimension);
  }

  void ImplementGeometry(const Convex&, void*) final {
    // Nothing to adjust.
  }

  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final {
    ReifyData* data = static_cast<ReifyData*>(user_data);
    double max_dimension = std::max(cylinder.radius(), cylinder.length() * 0.5);
    CheckTooSmall(data, max_dimension);
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) final {
    ReifyData* data = static_cast<ReifyData*>(user_data);
    double max_dimension = std::max(
        ellipsoid.a(), std::max(ellipsoid.b(), ellipsoid.c()));
    CheckTooSmall(data, max_dimension);
  }

  void ImplementGeometry(const HalfSpace&, void*) final {
    // Halfspaces are plenty big by definition; nothing to do.
  }

  void ImplementGeometry(const Sphere& sphere, void* user_data) final {
    ReifyData* data = static_cast<ReifyData*>(user_data);
    CheckTooSmall(data, sphere.radius());
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
  /*bool is_too_small = */GetShapeAdjuster()->MakeShapeAdjustments(
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

  // XXX removal causes too much collateral damage. Disable for now to see a
  // different set of damage.
  // if (is_too_small) {
  if (false) {
    geometry_state->RemoveRole(gid_source_id, gid, Role::kProximity);
  } else {
    geometry_state->AssignRole(gid_source_id, gid, props,
                               RoleAssign::kReplace);
  }
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
