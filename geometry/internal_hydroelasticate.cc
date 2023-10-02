#include "drake/geometry/internal_hydroelasticate.h"

#include <algorithm>
#include <string>

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
    properties->UpdateProperty(
        group_name, name,
        properties->GetPropertyOrDefault(group_name, name, default_value));
  };

  backfill(kHydroGroup, kElastic,
           config.hydroelastication.default_hydroelastic_modulus);
  backfill(kHydroGroup, kRezHint,
           config.hydroelastication.default_resolution_hint);
  backfill(kHydroGroup, kSlabThickness,
           config.hydroelastication.default_slab_thickness);

  backfill(kMaterialGroup, kHcDissipation,
           config.hydroelastication.default_hunt_crossley_dissipation);
  // Obey the documented law of CoulombFriction:
  // static friction >= dynamic friction.
  multibody::CoulombFriction friction{
    std::max(config.hydroelastication.default_dynamic_friction,
             config.hydroelastication.default_static_friction),
    config.hydroelastication.default_dynamic_friction,
  };
  backfill(kMaterialGroup, kFriction, friction);
}

class ShapeAdjuster final : public ShapeReifier {
 public:
  struct ReifyData {
    ProximityProperties* props{};
    bool is_too_small{false};
  };

  // @returns true if the shape is too small to participate in hydroelastic
  // contact for this scene.
  bool MakeShapeAdjustments(const Shape& shape, ProximityProperties* props) {
    ReifyData data = {props, false};
    shape.Reify(this, &data);
    return data.is_too_small;
  }

  using ShapeReifier::ImplementGeometry;

  void CheckTooSmall(ReifyData* data, double max_radius) {
    double rez_hint = data->props->GetPropertyOrDefault(
        kHydroGroup, kRezHint, 0.0);
    if (2 * max_radius < rez_hint * 1e-2) {
      data->is_too_small = true;
    }
  }

  void AdjustMesh(ReifyData* data, const std::string& extension) {
    // For now, let us claim that anything supplied as a mesh will likely be
    // appropriately sized to be part of the scene, and not a point-contact
    // helper shape. Therefore, too-small checking should not be
    // necessary. TODO(rpoyner-tri): Revisit this if there are
    // counter-examples.
    if (extension != ".vtk") {
      // We have no prayer of making a soft geometry -- avoid it.
      data->props->UpdateProperty(kHydroGroup, kComplianceType,
                                  HydroelasticType::kRigid);
    }
  }

  void ImplementGeometry(const Box& box, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius =
        std::max(box.width(), std::max(box.depth(), box.height()));
    CheckTooSmall(data, max_radius);
  }

  void ImplementGeometry(const Capsule& capsule, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius = capsule.length() * 0.5 + capsule.radius();
    CheckTooSmall(data, max_radius);
  }

  void ImplementGeometry(const Convex& convex, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    AdjustMesh(data, convex.extension());
  }

  void ImplementGeometry(const Cylinder& cylinder, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius = std::max(cylinder.radius(), cylinder.length() * 0.5);
    CheckTooSmall(data, max_radius);
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius = std::max(
        ellipsoid.a(), std::max(ellipsoid.b(), ellipsoid.c()));
    CheckTooSmall(data, max_radius);
  }

  void ImplementGeometry(const HalfSpace&, void*) final {
    // Halfspaces are plenty big by definition; nothing to do.
  }

  void ImplementGeometry(const Sphere& sphere, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    CheckTooSmall(data, sphere.radius());
  }

  void ImplementGeometry(const Mesh& mesh, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    AdjustMesh(data, mesh.extension());
  }
};

ShapeAdjuster* GetShapeAdjuster() {
  static ShapeAdjuster shape_adjuster;
  return &shape_adjuster;
}

}  // namespace

template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config) {
  DRAKE_DEMAND(geometry_state != nullptr);
  auto gids = geometry_state->GetGeometryIds(
      GeometrySet(geometry_state->GetAllGeometryIds()), Role::kProximity);
  for (const auto& gid : gids) {
    Hydroelasticate(geometry_state, config, gid);
  }
}

template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config, GeometryId geometry_id) {
  DRAKE_DEMAND(geometry_state != nullptr);
  auto gid = geometry_id;
  auto found_props = geometry_state->GetProximityProperties(gid);
  if (found_props == nullptr) {
    return;
  }
  // Get current proximity properties -- some should exist, given the
  // filtering done above.
  ProximityProperties props(*found_props);
  const HydroelasticType type = props.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);
  // Update proximity properties to a suitable hydro configuration.
  if (type == HydroelasticType::kUndefined) {
    // Start from the assumption that the shape can be declared soft.
    props.UpdateProperty(kHydroGroup, kComplianceType,
                         HydroelasticType::kSoft);
    BackfillDefaults(&props, config);

    // Shape adjuster will fix configurations that can't work.
    bool is_too_small = GetShapeAdjuster()->MakeShapeAdjustments(
        geometry_state->GetShape(gid), &props);
    // Jump through pointless hoops.
    SourceId gid_source_id;
    auto source_ids = geometry_state->GetAllSourceIds();
    for (const auto& source_id : source_ids) {
      if (geometry_state->BelongsToSource(gid, source_id)) {
        gid_source_id = source_id;
      }
    }
    if (is_too_small) {
      geometry_state->RemoveRole(gid_source_id, gid, Role::kProximity);
    } else {
      geometry_state->AssignRole(gid_source_id, gid, props,
                                 RoleAssign::kReplace);
    }
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
