/** @file
 Provides a set of functions to facilitate visualization operations based on
 geometry world state. */

#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_viewer_load_robot.hpp"

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
 be invoked _after_ registration is complete, but before context allocation.
 @param scene_graph    The system whose geometry will be sent in an LCM message.
 @throws std::logic_error if the system has already had its context allocated.
 */
void DispatchLoadMessage(const SceneGraph<double>& scene_graph);

}  // namespace geometry
}  // namespace drake
