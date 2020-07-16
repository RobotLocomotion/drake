#pragma once

#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
namespace drake {
namespace examples {
namespace mass_spring_system {

/// Expresses a mass-spring system's visualization geometry to a SceneGraph.
///
/// @system
/// name: MassSpringSystemGeometry
/// input_ports:
/// - state
/// output_ports:
/// - geometry_pose
/// @endsystem
///
/// The visualization shows the mass points as spheres.
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class MassSpringSystemGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MassSpringSystemGeometry);

  /// Creates, adds, and connects a MassSpringSystemGeometry system into the
  /// given `builder`.  Both the `mass_spring_system_state_port.get_system()`
  /// and `scene_graph` systems must have been added to the given `builder`
  /// already.
  ///
  /// The mass_spring_system_state_port is a N*3*2 state vector [x, v], where N
  /// is the number of mass points and x and v are the positions and velocities
  /// of the mass points respectively.
  ///
  /// The `scene_graph` pointer is not retained by the %MassSpringSystemGeometry
  /// system. The return value pointer is an alias of the new
  /// %MassSpringSystemGeometry system that is owned by the `builder`.
  static const MassSpringSystemGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& mass_spring_system_state_port,
      geometry::SceneGraph<double>* scene_graph, int num_points);

 private:
  explicit MassSpringSystemGeometry(geometry::SceneGraph<double>*,
                                    int num_points);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_;
  // The identifier for each point's frame.
  std::vector<geometry::FrameId> frame_ids_;
  // Number of mass points.
  int num_points_;
};

}  // namespace mass_spring_system
}  // namespace examples
}  // namespace drake
