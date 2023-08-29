#include "drake/geometry/hydroelastize.h"

#include <string>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

using internal::HydroelasticType;
using internal::kComplianceType;
using internal::kElastic;
using internal::kHydroGroup;
using internal::kRezHint;
using internal::kSlabThickness;

namespace {

void BackfillDefaults(ProximityProperties* properties) {
  auto backfill = [&](const std::string& group_name, const std::string& name,
                      auto default_value) {
    properties->UpdateProperty(
        group_name, name,
        properties->GetPropertyOrDefault(group_name, name, default_value));
  };
  backfill(kHydroGroup, kElastic, 1e8);
  backfill(kHydroGroup, kRezHint, 0.01);
  backfill(kHydroGroup, kSlabThickness, 1e3);
}

template <typename Reader>
class ShapeAdjuster final : public ShapeReifier {
 public:
  explicit ShapeAdjuster(const Reader& inspector)
      : inspector_(inspector) {}

  void MakeShapeAdjustments(GeometryId gid, ProximityProperties* props) {
    const auto& shape(inspector_.GetShape(gid));
    shape.Reify(this, props);
  }

  void ImplementGeometry(const Box&, void*) final {}
  void ImplementGeometry(const Capsule&, void*) final {}
  void ImplementGeometry(const Convex&, void*) final {}
  void ImplementGeometry(const Cylinder&, void*) final {}
  void ImplementGeometry(const Ellipsoid&, void*) final {}
  void ImplementGeometry(const HalfSpace&, void*) final {}
  void ImplementGeometry(const MeshcatCone&, void*) final {}
  void ImplementGeometry(const Sphere&, void*) final {}
  void ImplementGeometry(const Mesh& mesh, void* user_data) final {
    auto* props = static_cast<ProximityProperties*>(user_data);
    if (mesh.extension() != ".vtk") {
      // We have no prayer of making a soft geometry -- avoid it.
      props->UpdateProperty(kHydroGroup, kComplianceType,
                            HydroelasticType::kRigid);
    }
  }

 private:
  const Reader& inspector_;
};

}  // namespace

template <typename T>
void Hydroelastize(SceneGraph<T>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);
  auto& inspector = scene_graph->model_inspector();
  Hydroelastize<T, SceneGraphInspector<T>, SceneGraph<T>>(
      inspector, scene_graph);
}

template <typename T>
void Hydroelastize(GeometryState<T>* geometry_state) {
  DRAKE_DEMAND(geometry_state != nullptr);
  Hydroelastize<T, GeometryState<T>, GeometryState<T>>(
      *geometry_state, geometry_state);
}

template <typename T, typename Reader, typename Writer>
void Hydroelastize(const Reader& reader, Writer* writer) {
  // Iterate over all geometries with a proximity role.
  auto gids = reader.GetGeometryIds(
      GeometrySet(reader.GetAllGeometryIds()), Role::kProximity);
  auto source_ids = reader.GetAllSourceIds();
  ShapeAdjuster shape_adjuster(reader);
  for (const auto& gid : gids) {
    // Get current proximity properties -- some should exist, given the
    // filtering done above.
    ProximityProperties props(*reader.GetProximityProperties(gid));
    const HydroelasticType type = props.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);
    // Update proximity properties to a suitable hydro configuration.
    if (type == HydroelasticType::kUndefined) {
      // Start from the assumption that the shape can be declared soft.
      props.UpdateProperty(kHydroGroup, kComplianceType,
                           HydroelasticType::kSoft);
      BackfillDefaults(&props);

      // Shape adjuster will fix configurations that can't work.
      shape_adjuster.MakeShapeAdjustments(gid, &props);
      // Jump through pointless hoops.
      SourceId gid_source_id;
      for (const auto& source_id : source_ids) {
        if (reader.BelongsToSource(gid, source_id)) {
          gid_source_id = source_id;
        }
      }
      writer->AssignRole(gid_source_id, gid, props, RoleAssign::kReplace);
    }
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<void(*)(SceneGraph<T>*)>(&Hydroelastize),
    static_cast<void(*)(GeometryState<T>*)>(&Hydroelastize)
))

}  // namespace geometry
}  // namespace drake
