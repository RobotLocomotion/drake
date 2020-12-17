#pragma once

#include "drake/examples/rimless_wheel/gen/rimless_wheel_params.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

/// Expresses a RimlessWheel's geometry to a SceneGraph.
///
/// @system
/// name: RimlessWheelGeometry
/// input_ports:
/// - floating_base_state
/// output_ports:
/// - geometry_pose
/// @endsystem
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class RimlessWheelGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RimlessWheelGeometry);
  ~RimlessWheelGeometry() final;

  /// Creates, adds, and connects a RimlessWheelGeometry system into the given
  /// `builder`.  Both the `floating_base_state_port.get_system()`
  /// and `scene_graph` systems must have been added to the given `builder`
  /// already.  The `rimless_wheel_params` sets the parameters of the
  /// geometry registered with `scene_graph`.
  ///
  /// The `scene_graph` pointer is not retained by the %RimlessWheelGeometry
  /// system.  The return value pointer is an alias of the new
  /// %RimlessWheelGeometry system that is owned by the `builder`.
  static const RimlessWheelGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& floating_base_state_port,
      const RimlessWheelParams<double>& rimless_wheel_params,
      geometry::SceneGraph<double>* scene_graph);

  /// Creates, adds, and connects a RimlessWheelGeometry system into the given
  /// `builder`.  Both the `floating_base_state_port.get_system()` and
  /// `scene_graph` systems must have been added to the given `builder` already.
  /// RimlessWheelParams are set to their default values.
  ///
  /// The `scene_graph` pointer is not retained by the %RimlessWheelGeometry
  /// system.  The return value pointer is an alias of the new
  /// %RimlessWheelGeometry system that is owned by the `builder`.
  static const RimlessWheelGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& floating_base_state_port,
      geometry::SceneGraph<double>* scene_graph) {
    return AddToBuilder(builder, floating_base_state_port,
                        RimlessWheelParams<double>(), scene_graph);
  }

 private:
  RimlessWheelGeometry(const RimlessWheelParams<double>& rimless_wheel_params,
                       geometry::SceneGraph<double>*);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_{};
  geometry::FrameId frame_id_{};
};

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
