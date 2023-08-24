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

template <typename T>
class ShapeAdjuster final : public ShapeReifier {
 public:
  explicit ShapeAdjuster(const SceneGraphInspector<T>& inspector)
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
  const SceneGraphInspector<T>& inspector_;
};

}  // namespace

template <typename T>
void Hydroelastize(SceneGraph<T>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);
  // Iterate over all geometries with a proximity role.
  auto& inspector = scene_graph->model_inspector();
  auto gids = inspector.GetGeometryIds(
      GeometrySet(inspector.GetAllGeometryIds()), Role::kProximity);
  auto source_ids = inspector.GetAllSourceIds();
  ShapeAdjuster shape_adjuster(inspector);
  for (const auto& gid : gids) {
    // Get current proximity properties -- some should exist, given the
    // filtering done above.
    ProximityProperties props(*inspector.GetProximityProperties(gid));
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
        if (inspector.BelongsToSource(gid, source_id)) {
          gid_source_id = source_id;
        }
      }
      scene_graph->AssignRole(gid_source_id, gid, props, RoleAssign::kReplace);
    }
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&Hydroelastize<T>));

}  // namespace geometry
}  // namespace drake
