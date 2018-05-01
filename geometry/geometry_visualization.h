/** @file
 Provides a set of functions to facilitate visualization operations based on
 geometry system state. */

#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm_interface.h"
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

/** Dispatches an LCM load message based on the registered geometry. It should
  be invoked _after_ registration is complete, but before context allocation. */
void DispatchLoadMessage(const SceneGraph<double>& scene_graph,
                         lcm::DrakeLcmInterface* lcm);

/** Extends the diagram with the required components to interface with
 drake_visualizer.
 @param scene_graph  The system whose geometry will be visualized.
 @param builder      The diagram builder to which the system belongs; additional
                     systems will be added to enable visualization updates.
 @param lcm          The lcm interface through which lcm messages will be
                     dispatched. */
void ConnectVisualization(const SceneGraph<double>& scene_graph,
                          systems::DiagramBuilder<double>* builder,
                          lcm::DrakeLcmInterface* lcm);

}  // namespace geometry
}  // namespace drake
