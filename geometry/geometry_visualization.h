/** @file
 Provides a set of functions to facilitate visualization operations based on
 SceneGraph system state. */

#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

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

/** Extends a Diagram with the required components to interface with
 drake_visualizer. This must be called _during_ Diagram building and
 uses the given `builder` to add relevant subsystems and connections.

 This is a convenience method to simplify some common boilerplate for adding
 visualization capability to a Diagram. What it does is:

 - adds an initialization event that sends the required load message to set up
   the visualizer with the relevant geometry,
 - adds systems PoseBundleToDrawMessage and LcmPublisherSystem to
   the Diagram and connects the draw message output to the publisher input,
 - connects the `scene_graph` pose bundle output to the PoseBundleToDrawMessage
   system, and
 - sets the publishing rate to 1/60 of a second (simulated time).

 @note The initialization event occurs when Simulator::Initialize() is called
 (explicitly or implicitly at the start of a simulation). If you aren't going
 to be using a Simulator, use DispatchLoadMessage() to send the message
 yourself.

 @param builder      The diagram builder being used to construct the Diagram.
 @param scene_graph  The System in `builder` containing the geometry to be
                     visualized.
 @param lcm          An optional lcm interface through which lcm messages will
                     be dispatched. Will be allocated internally if none is
                     supplied.

 @pre This method has not been previously called while building the
      builder's current Diagram.
 @pre The given `scene_graph` must be contained within the supplied
      DiagramBuilder.

 @returns the LcmPublisherSystem (in case callers, e.g., need to change the
 default publishing rate).

 @see geometry::DispatchLoadMessage()
 @ingroup visualization
 */
systems::lcm::LcmPublisherSystem* ConnectDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm = nullptr);

/** Implements ConnectDrakeVisualizer, but using @p pose_bundle_output_port to
 explicitly specify the output port used to get pose bundles for
 @p scene_graph.  This is required, for instance, when the SceneGraph is
 inside a Diagram, and the Diagram exports the pose bundle port.

 @pre pose_bundle_output_port must be connected directly to the
 pose_bundle_output_port of @p scene_graph.

 @see ConnectDrakeVisualizer().
 */
systems::lcm::LcmPublisherSystem* ConnectDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const SceneGraph<double>& scene_graph,
    const systems::OutputPort<double>& pose_bundle_output_port,
    lcm::DrakeLcmInterface* lcm = nullptr);

/** (Advanced) Explicitly dispatches an LCM load message based on the registered
 geometry. Normally this is done automatically at Simulator initialization. But
 if you have to do it yourself (likely because you are not using a Simulator),
 it should be invoked _after_ registration is complete. Typically this is used
 after ConnectDrakeVisualizer() has been used to add visualization to the
 Diagram that contains the given `scene_graph`. The message goes to
 LCM channel "DRAKE_VIEWER_LOAD_ROBOT".

 @see geometry::ConnectDrakeVisualizer() */
void DispatchLoadMessage(
    const SceneGraph<double>& scene_graph, lcm::DrakeLcmInterface* lcm);

}  // namespace geometry
}  // namespace drake
