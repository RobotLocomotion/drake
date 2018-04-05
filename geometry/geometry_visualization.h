/** @file
 Provides a set of functions to facilitate visualization operations based on
 geometry world state. */

#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {

// Simple class declared as a friend to GeometryState to facilitate the creation
// of visualization artifacts directly from the contents of GeometryState.
class GeometryVisualizationImpl {
 public:
  // Given an instance of GeometryState, returns an lcm message sufficient
  // to load the state's geometry.
  static lcmt_viewer_load_robot BuildLoadMessage(
      const GeometryState<double>& state);
};

}  // namespace internal
#endif  // DRAKE_DOXYGEN_CXX

/** Configures the diagram to interface with drake_visualizer. This should be
 invoked as the _last_ thing before building the diagram; registration of all
 frames and geometries should be done, but not context allocated.
 @param scene_graph  The system whose geometry will be sent in an LCM message.
 @param builder      The diagram builder to which the system belongs; additional
                     systems will be added to enable visualization updates.
 @throws std::logic_error if the system has already had its context allocated.
 */
void ConfigureVisualization(const SceneGraph<double>& scene_graph,
                            systems::DiagramBuilder<double>* builder);

}  // namespace geometry
}  // namespace drake
