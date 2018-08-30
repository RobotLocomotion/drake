/** @file
 Provides a set of functions to facilitate visualization operations based on
 SceneGraph system state. */

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

/** Extends a Diagram with the required components to interface with
 drake_visualizer, and connects a source System output to be visualized.
 This must be called _during_ Diagram building and uses the given `builder` to
 add relevant subsystems and connections.

 This is a convenience method to simplify some common boilerplate for adding
 visualization capability to a Diagram. What it does is:
 - invokes ConnectVisualization() to extend the Diagram, and
 - connects a pose-providing output port of a System to the appropriate input
   port of the `scene_graph`.

 @warning You must only call this function _once_ while building a Diagram.

 @note This convenience method assumes you have just one System that is
 providing runtime poses. If there are multiple Systems registered with the
 same SceneGraph, you can add more yourself after this call using:
 @code
   builder->Connect(pose_output_port,
                    scene_graph.get_source_pose_port(source_id));
 @endcode
 Add one Connect() call for each additional registered System.

 @note This function allocates an lcm::DrakeLcm object internally and maintains
 it hidden in the Diagram. If you need access to that yourself, allocate one
 yourself and then call ConnectVisualization() directly instead of this method.

 @param builder      The diagram builder being used to construct the Diagram.
 @param scene_graph  The System in `builder` containing the geometry to be
                     visualized.
 @param source_id    The source id of the System that will supply poses at
                     runtime to position the SceneGraph geometry.
 @param pose_output_port
                     The output port of the source System from which poses
                     can be obtained at runtime.

 @pre Neither this method nor ConnectVisualization() has been called before
      while building the current Diagram.
 @pre The given `pose_output_port` _must_ belong to the System whose `source_id`
      is given.
 @pre Both Systems mentioned above must be contained within the supplied
      DiagramBuilder.

 @see geometry::ConnectVisualization() */
void AddVisualization(systems::DiagramBuilder<double>* builder,
                      const SceneGraph<double>& scene_graph,
                      geometry::SourceId source_id,
                      const systems::OutputPort<double>& pose_output_port);

/** Extends a Diagram with the required components to interface with
 drake_visualizer. Prefer AddVisualization() if you don't need to allocate your
 own lcm::DrakeLcm object. This must be called _during_ Diagram building and
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

 You can then connect source output ports for visualization like this:
 @code
   builder->Connect(pose_output_port,
                    scene_graph.get_source_pose_port(source_id));
 @endcode

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

 @pre Neither this method nor AddVisualization() has been called before
      while building the current Diagram.
 @pre The given `scene_graph` must be contained within the supplied
      DiagramBuilder.

 @see geometry::AddVisualization(), geometry::DispatchLoadMessage() */
void ConnectVisualization(systems::DiagramBuilder<double>* builder,
                          const SceneGraph<double>& scene_graph,
                          lcm::DrakeLcmInterface* lcm = nullptr);

/** Explicitly dispatches an LCM load message based on the registered geometry.
 Normally this is done automatically at Simulator initialization. But if you
 have to do it yourself (likely because you are not using a Simulator), it
 should be invoked _after_ registration is complete. Typically this is used
 after ConnectVisualization() has been used to add visualization to the
 Diagram that contains the given `scene_graph`.

 @see geometry::ConnectVisualization() */
void DispatchLoadMessage(const SceneGraph<double>& scene_graph,
                         lcm::DrakeLcmInterface* lcm);

}  // namespace geometry
}  // namespace drake
