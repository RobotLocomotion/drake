#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace particles {

/// Expresses a Particle system's visualization geometry to a SceneGraph.
///
/// @system
/// name: ParticleGeometry
/// input_ports:
/// - state
/// output_ports:
/// - geometry_pose
/// @endsystem
///
/// The visualization shows the particle as a sphere moving along the x axis.
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class ParticleGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParticleGeometry);

  /// Creates, adds, and connects a ParticleGeometry system into the given
  /// `builder`.  Both the `particle_state_port.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// The particle_state_port is a 2D state vector [q, qdot].
  ///
  /// The `scene_graph` pointer is not retained by the %ParticleGeometry system.
  /// The return value pointer is an alias of the new %ParticleGeometry system
  /// that is owned by the `builder`.
  static const ParticleGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& particle_state_port,
      geometry::SceneGraph<double>* scene_graph);

 private:
  explicit ParticleGeometry(geometry::SceneGraph<double>*);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_;
  // The identifier for the particle frame.
  geometry::FrameId frame_id_;
};

}  // namespace particles
}  // namespace examples
}  // namespace drake
