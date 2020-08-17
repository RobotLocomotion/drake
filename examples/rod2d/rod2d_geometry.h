#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace rod2d {

// Expresses the Rod2D's geometry to a SceneGraph.
///
/// @system
/// name: Rod2dGeometry
/// input_ports:
/// - state
/// output_ports:
/// - geometry_pose
/// @endsystem
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class Rod2dGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Rod2dGeometry);
  ~Rod2dGeometry() final;

  /// Creates, adds, and connects a Rod2dGeometry system into the given
  /// `builder`. Both the `rod2d_state_port.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// The `scene_graph` pointer is not retained by the %Rod2dGeometry
  /// system. The return value pointer is an alias of the new %Rod2dGeometry
  /// system that is owned by the `builder`.
  ///
  /// The rod is visualized with a cylinder with the given `radius`, and
  /// `length` displayed with the a default grey color.
  static const Rod2dGeometry* AddToBuilder(
      double radius, double length, systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& rod2d_state_port,
      geometry::SceneGraph<double>* scene_graph);

 private:
  Rod2dGeometry(double radius, double length, geometry::SceneGraph<double>*);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_{};
  // The id for the rod's body.
  geometry::FrameId frame_id_{};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
