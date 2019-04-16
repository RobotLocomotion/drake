#pragma once

#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {

/// Expresses a PendulumPlants's geometry to a SceneGraph.
///
/// @system{PendulumGeometry,
///    @input_port{state},
///    @output_port{geometry_pose}
/// }
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class PendulumGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PendulumGeometry);

  /// Creates, adds, and connects a PendulumGeometry into the given builder.
  static const PendulumGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const PendulumPlant<double>& pendulum_plant,
      geometry::SceneGraph<double>* scene_graph);

 private:
  explicit PendulumGeometry(geometry::SceneGraph<double>*);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_;
  // The id for the pendulum (arm + point mass) frame.
  geometry::FrameId frame_id_;
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
