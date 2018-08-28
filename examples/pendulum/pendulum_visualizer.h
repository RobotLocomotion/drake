#pragma once

#include <memory>

#include "drake/examples/pendulum/gen/pendulum_params.h"
#include "drake/examples/pendulum/gen/pendulum_state.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {

/// A simple system that registers geometry (for visualization) with a
/// geometry::SceneGraph and translates a PendulumState (e.g. from the
/// PendulumPlant output) into a geometry::FramePoseVector (for input into
/// the geometry::SceneGraph).
class PendulumVisualizer : public systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PendulumVisualizer);

  /// Create the visualizer and register the geometry with the scene_graph.
  PendulumVisualizer(const PendulumParams<double>& params,
                     geometry::SceneGraph<double>* scene_graph);

  /// Create the visualizer with the default PendulumParameters and register
  // the geometry with the scene_graph.
  PendulumVisualizer(geometry::SceneGraph<double>* scene_graph) :
      PendulumVisualizer(PendulumParams<double>(), scene_graph) {};

  ~PendulumVisualizer() override {};


  /// Returns the input port expecting a PendulumState<double>.
  const systems::InputPort<double>& get_state_input_port() const;


private:

};

/// Constructs a PendulumVisualizer, a SceneGraph, and the connections to
/// visualization.  Also calls geometry::DispatchLoadMessage.
/// @returns pointer to the new PendulumVisualizer to be wired into the
/// remainder of the diagram.
PendulumVisualizer* AddPendulumVisualizerAndPublisher(
    const PendulumParams<double>& params,
    systems::DiagramBuilder<double>* builder, lcm::DrakeLcmInterface* lcm);

/// Constructs a PendulumVisualizer (with the default parameters), a SceneGraph,
/// and the connections to visualization.  Also calls
/// geometry::DispatchLoadMessage.
/// @returns pointer to the new PendulumVisualizer to be wired into the
/// remainder of the diagram.
PendulumVisualizer* AddPendulumVisualizerAndPublisher(
    systems::DiagramBuilder<double> *builder, lcm::DrakeLcmInterface *lcm);

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
