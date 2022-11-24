#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace geometry {

namespace internal {
template <typename T>
std::unordered_map<GeometryId, GeometryId> CloneGeometriesToPlant(
    const std::set<GeometryId>& ids_src,
    const multibody::MultibodyPlant<T>& plant_src,
    const SceneGraph<T>& scene_graph_src,
    multibody::MultibodyPlant<T>* plant_dest,
    const multibody::internal::MultibodyElementAccessor<T, T>& accessor);
}  // namespace internal
}  // namespace geometry
}  // namespace drake
