#include "drake/geometry/scene_graph_config_functions.h"

namespace drake {
namespace geometry {

void ApplySceneGraphConfig(const SceneGraphConfig& config,
                           SceneGraph<double>* scene_graph) {
  scene_graph->set_hydroelastize(config.hydroelastize);
}

}  // namespace geometry
}  // namespace drake
