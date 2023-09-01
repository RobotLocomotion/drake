#include "drake/geometry/hydroelastize.h"

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

  // TODO(rpoyner-tri): implement too-small checks for all shapes.
  void ImplementGeometry(const Box&, void*) final {}
  void ImplementGeometry(const Capsule&, void*) final {}
  void ImplementGeometry(const Convex&, void*) final {}
  void ImplementGeometry(const Cylinder&, void*) final {}
  void ImplementGeometry(const Ellipsoid&, void*) final {}
  void ImplementGeometry(const HalfSpace&, void*) final {}
  void ImplementGeometry(const MeshcatCone&, void*) final {}
  void ImplementGeometry(const Sphere& sphere, void* ptr) final {
    ReifyData* data = static_cast<ReifyData*>(ptr);
    double rez_hint = data->props->GetPropertyOrDefault(
        kHydroGroup, kRezHint, 0.0);
    if (sphere.radius() < rez_hint * 1e-2) {
      data->is_too_small = true;
    }
  }
  void ImplementGeometry(const Mesh& mesh, void* ptr) final {
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
        // TODO(rpoyner-tri): figure out a less hostile disposition of
        // too-small geometries. This particular implementation violates
        // weird/sketchy caching assumptions of MbP and in particular,
        // internal::GeometryNames. It's been hacked for now (see XXX markers
        // there), but perhaps a solution involving collision filters would be
        // better.
        geometry_state->RemoveGeometry(gid_source_id, gid);
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
