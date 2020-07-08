#pragma once

#include "drake/examples/compass_gait/gen/compass_gait_params.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace compass_gait {

/// Expresses a CompassGait's geometry to a SceneGraph.
///
/// @system
/// name: CompassGaitGeometry
/// input_ports:
/// - floating_base_state
/// output_ports:
/// - geometry_pose
/// @endsystem
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class CompassGaitGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompassGaitGeometry);
  ~CompassGaitGeometry() final;

  /// Creates, adds, and connects a CompassGaitGeometry system into the given
  /// `builder`.  Both the `floating_base_state_port.get_system()`
  /// and `scene_graph` systems must have been added to the given `builder`
  /// already.  The `compass_gait_params` sets the parameters of the
  /// geometry registered with `scene_graph`; the visualization changes
  /// based on the leg length and the ration of leg mass to hip mass (the leg
  /// mass sphere is scaled assuming a constant density).
  ///
  /// The `scene_graph` pointer is not retained by the %CompassGaitGeometry
  /// system.  The return value pointer is an alias of the new
  /// %CompassGaitGeometry system that is owned by the `builder`.
  static const CompassGaitGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& floating_base_state_port,
      const CompassGaitParams<double>& compass_gait_params,
      geometry::SceneGraph<double>* scene_graph);

  /// Creates, adds, and connects a CompassGaitGeometry system into the given
  /// `builder`.  Both the `floating_base_state_port.get_system()` and
  /// `scene_graph` systems must have been added to the given `builder` already.
  /// CompassGaitParams are set to their default values.
  ///
  /// The `scene_graph` pointer is not retained by the %CompassGaitGeometry
  /// system.  The return value pointer is an alias of the new
  /// %CompassGaitGeometry system that is owned by the `builder`.
  static const CompassGaitGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& floating_base_state_port,
      geometry::SceneGraph<double>* scene_graph) {
    return AddToBuilder(builder, floating_base_state_port,
                        CompassGaitParams<double>(), scene_graph);
  }

 private:
  CompassGaitGeometry(const CompassGaitParams<double>& compass_gait_params,
                      geometry::SceneGraph<double>*);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_{};
  geometry::FrameId left_leg_frame_id_{};
  geometry::FrameId right_leg_frame_id_{};
};

}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
