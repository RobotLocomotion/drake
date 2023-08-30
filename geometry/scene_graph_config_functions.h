#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/geometry/scene_graph_config.h"

namespace drake {
namespace geometry {

/// Applies settings given in `config` to an existing `scene_graph`.
void ApplySceneGraphConfig(const SceneGraphConfig& config,
                               SceneGraph<double>* scene_graph);

}  // namespace geometry
}  // namespace drake
