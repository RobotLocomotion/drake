#include "drake/geometry/geometry_cloning.h"
#include "drake/multibody/parsing/scoped_names.h"


namespace drake {
namespace geometry {

namespace internal {
template <typename T>
std::unordered_map<GeometryId, GeometryId> CloneGeometriesToPlant(
    const std::set<GeometryId>& ids_src,
    const multibody::MultibodyPlant<T>& plant_src,
    const SceneGraph<T>& scene_graph_src,
    multibody::MultibodyPlant<T>* plant_dest,
    const multibody::internal::MultibodyElementAccessor<T,T>& accessor) {
  const SceneGraphInspector<T>& inspector_src =
      scene_graph_src.model_inspector();
  std::unordered_map<GeometryId, GeometryId> id_map;

  for (const auto& src_id : ids_src) {

    const auto frame_id_src = inspector_src.GetFrameId(src_id);
    const multibody::Body<T>* body_src =
        plant_src.GetBodyFromFrameId(frame_id_src);
    DRAKE_DEMAND(body_src != nullptr);
    const multibody::Body<T>& body_dest = accessor.get_variant(*body_src);

    std::unique_ptr<GeometryInstance> geometry_instance_dest =
        inspector_src.CloneGeometryInstance(src_id);

    // Use new "scoped" name.
    std::string prefix_src;
    if (body_src->model_instance() != multibody::world_model_instance()) {
      prefix_src = plant_src.GetModelInstanceName(body_src->model_instance());
    }

    std::string prefix_dest;
    if (body_dest.model_instance() != multibody::world_model_instance()) {
      prefix_dest =
          plant_dest->GetModelInstanceName(body_dest.model_instance());
    }

    auto scoped_name_src =
        multibody::parsing::ParseScopedName(geometry_instance_dest->name());
    DRAKE_DEMAND(scoped_name_src.instance_name == prefix_src);

    auto scoped_name_dest =
        multibody::parsing::PrefixName(prefix_dest, scoped_name_src.name);

    geometry_instance_dest->set_name(scoped_name_dest);

    // TODO(eric.cousineau): How to relax this constraint? How can we
    // register with SceneGraph only?
    // See: https://github.com/RobotLocomotion/drake/issues/13445
    // TODO(eric.cousineau): Try Ale's potential fix here:
    // https://github.com/RobotLocomotion/drake/pull/13371
    geometry::GeometryId geometry_id_dest = plant_dest->RegisterGeometry(
        body_dest, std::move(geometry_instance_dest));
    id_map[src_id] = geometry_id_dest;
  }

  auto collision_pairs = inspector_src.GetCollisionFilteredPairs();

  for (const auto& p : collision_pairs) {
    if (ids_src.count(p.first) == 1 && ids_src.count(p.second) == 1) {
      auto id_a_dest = id_map[p.first];
      auto id_b_dest = id_map[p.second];

      plant_dest->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
          {"a", geometry::GeometrySet(id_a_dest)},
          {"b", geometry::GeometrySet(id_b_dest)});
    }
  }

  return id_map;
}
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&CloneGeometriesToPlant<T>))
}  // namespace internal
}  // namespace geometry
}  // namespace drake
