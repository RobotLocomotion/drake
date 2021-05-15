#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace quadrotor {

/// Expresses a QuadrotorPlant's geometry to a SceneGraph.
///
/// @system
/// name: QuadrotorGeometry
/// input_ports:
/// - state
/// output_ports:
/// - geometry_pose
/// @endsystem
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class QuadrotorGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadrotorGeometry);
  ~QuadrotorGeometry() final;

  /// Returns the frame of the geometry registered with a SceneGraph.  This can
  /// be useful, e.g., if one would like to add a camera to the quadrotor.
  geometry::FrameId get_frame_id() const { return frame_id_; }

  /// Creates, adds, and connects a QuadrotorGeometry system into the given
  /// `builder`.  Both the `quadrotor_state.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// The `scene_graph` pointer is not retained by the %QuadrotorGeometry
  /// system.  The return value pointer is an alias of the new
  /// %QuadrotorGeometry system that is owned by the `builder`.
  static const QuadrotorGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& quadrotor_state_port,
      geometry::SceneGraph<double>* scene_graph);

 private:
  explicit QuadrotorGeometry(geometry::SceneGraph<double>*);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_{};
  // The id for the quadrotor body.
  geometry::FrameId frame_id_{};
};

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
