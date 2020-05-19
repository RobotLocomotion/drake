#include "drake/geometry/scene_graph_inspector.h"

#include <memory>

namespace drake {
namespace geometry {

template <typename T>
std::unique_ptr<GeometryInstance>
SceneGraphInspector<T>::CloneGeometryInstance(GeometryId id) const {
  const std::string name = GetName(id);
  const math::RigidTransformd X_PG = GetPoseInFrame(id);
  std::unique_ptr<Shape> shape = GetShape(id).Clone();
  auto geometry_instance = std::make_unique<GeometryInstance>(
      X_PG, std::move(shape), name);
  if (const auto* props = GetProximityProperties(id)) {
    geometry_instance->set_proximity_properties(*props);
  }
  if (const auto* props = GetIllustrationProperties(id)) {
    geometry_instance->set_illustration_properties(*props);
  }
  if (const auto* props = GetPerceptionProperties(id)) {
    geometry_instance->set_perception_properties(*props);
  }
  return geometry_instance;
}

// Explicitly instantiates on the most common scalar types.
template class SceneGraphInspector<double>;
template class SceneGraphInspector<AutoDiffXd>;

}  // namespace geometry
}  // namespace drake
