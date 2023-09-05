#include "drake/geometry/hydroelastize.h"

#include <algorithm>
#include <string>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"

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
           config.hydroelastize_default_hydroelastic_modulus);
  backfill(kHydroGroup, kRezHint,
           config.hydroelastize_default_resolution_hint);
  backfill(kHydroGroup, kSlabThickness,
           config.hydroelastize_default_slab_thickness);
  backfill(kMaterialGroup, kHcDissipation,
           config.hydroelastize_default_hunt_crossley_dissipation);
  backfill(kMaterialGroup, kFriction,
           config.hydroelastize_default_dynamic_friction);
}

class ShapeAdjuster final : public ShapeReifier {
 public:
  // @returns true if the shape is too small to participate in hydroelastic
  // contact for this scene.
  bool MakeShapeAdjustments(const Shape& shape, ProximityProperties* props) {
    ReifyData data = {props, false};
    shape.Reify(this, &data);
    return data.is_too_small;
  }

  void ImplementGeometry(const Box& box, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius =
        std::max(box.width(), std::max(box.depth(), box.height()));
    double rez_hint = data->props->GetPropertyOrDefault(
        kHydroGroup, kRezHint, 0.0);
    if (2 * max_radius < rez_hint * 1e-2) {
      data->is_too_small = true;
    }
  }

  void ImplementGeometry(const Capsule& capsule, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius = capsule.length() * 0.5 + capsule.radius();
    double rez_hint = data->props->GetPropertyOrDefault(
        kHydroGroup, kRezHint, 0.0);
    if (2 * max_radius < rez_hint * 1e-2) {
      data->is_too_small = true;
    }
  }

  void ImplementGeometry(const Convex&, void*) final {
    // For now, let us claim that anything supplied as a mesh will likely be
    // appropriately sized to be part of the scene, and not a point-contact
    // helper shape. Therefore, too-small checking should not be
    // necessary. TODO(rpoyner-tri): Revisit this if there are
    // counter-examples.
  }

  void ImplementGeometry(const Cylinder& cylinder, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius = std::max(cylinder.radius(), cylinder.length() * 0.5);
    double rez_hint = data->props->GetPropertyOrDefault(
        kHydroGroup, kRezHint, 0.0);
    if (2 * max_radius < rez_hint * 1e-2) {
      data->is_too_small = true;
    }
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double max_radius = std::max(
        ellipsoid.a(), std::max(ellipsoid.b(), ellipsoid.c()));
    double rez_hint = data->props->GetPropertyOrDefault(
        kHydroGroup, kRezHint, 0.0);
    if (2 * max_radius < rez_hint * 1e-2) {
      data->is_too_small = true;
    }
  }

  void ImplementGeometry(const HalfSpace&, void*) final {
    // Halfspaces are plenty big by definition; nothing to do.
  }

  void ImplementGeometry(const MeshcatCone&, void*) final {
    // Documented not to participate in proximity queries; nothing to do.
  }

  void ImplementGeometry(const Sphere& sphere, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double rez_hint = data->props->GetPropertyOrDefault(
        kHydroGroup, kRezHint, 0.0);
    if (2 * sphere.radius() < rez_hint * 1e-2) {
      data->is_too_small = true;
    }
  }

  void ImplementGeometry(const Mesh& mesh, void* ptr) final {
    // For now, let us claim that anything supplied as a mesh will likely be
    // appropriately sized to be part of the scene, and not a point-contact
    // helper shape. Therefore, too-small checking should not be
    // necessary. TODO(rpoyner-tri): Revisit this if there are
    // counter-examples.
    ReifyData* data = static_cast<ReifyData*>(ptr);
    if (mesh.extension() != ".vtk") {
      // We have no prayer of making a soft geometry -- avoid it.
      data->props->UpdateProperty(kHydroGroup, kComplianceType,
                                  HydroelasticType::kRigid);
    }
  }

  struct ReifyData {
    ProximityProperties* props{};
    bool is_too_small{false};
  };
};

}  // namespace

template <typename T>
void Hydroelastize(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config) {
  DRAKE_DEMAND(geometry_state != nullptr);
  auto gids = geometry_state->GetGeometryIds(
      GeometrySet(geometry_state->GetAllGeometryIds()), Role::kProximity);
  auto source_ids = geometry_state->GetAllSourceIds();
  ShapeAdjuster shape_adjuster;
  for (const auto& gid : gids) {
    // Get current proximity properties -- some should exist, given the
    // filtering done above.
    ProximityProperties props(*geometry_state->GetProximityProperties(gid));
    const HydroelasticType type = props.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);
    // Update proximity properties to a suitable hydro configuration.
    if (type == HydroelasticType::kUndefined) {
      // Start from the assumption that the shape can be declared soft.
      props.UpdateProperty(kHydroGroup, kComplianceType,
                           HydroelasticType::kSoft);
      BackfillDefaults(&props, config);

      // Shape adjuster will fix configurations that can't work.
      bool is_too_small = shape_adjuster.MakeShapeAdjustments(
          geometry_state->GetShape(gid), &props);
      // Jump through pointless hoops.
      SourceId gid_source_id;
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
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &Hydroelastize<T>
))
}  // namespace internal
}  // namespace geometry
}  // namespace drake
