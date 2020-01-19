#pragma once

#include "drake/examples/acrobot/gen/acrobot_params.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace acrobot {

/// Expresses an AcrobotPlant's geometry to a SceneGraph.
///
/// @system{AcrobotGeometry,
///    @input_port{state},
///    @output_port{geometry_pose}
/// }
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class AcrobotGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotGeometry);
  ~AcrobotGeometry() final;

  /// Creates, adds, and connects an AcrobotGeometry system into the given
  /// `builder`.  Both the `acrobot_state.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// @param acrobot_params sets the parameters of the geometry registered
  /// with `scene_graph`.
  ///
  /// The `scene_graph` pointer is not retained by the %AcrobotGeometry
  /// system.  The return value pointer is an alias of the new
  /// %AcrobotGeometry system that is owned by the `builder`.
  static const AcrobotGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& acrobot_state_port,
      const AcrobotParams<double>& acrobot_params,
      geometry::SceneGraph<double>* scene_graph);

  /// Creates, adds, and connects an AcrobotGeometry system into the given
  /// `builder`.  Both the `acrobot_state.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// Acrobot parameters are set to their default values.
  ///
  /// The `scene_graph` pointer is not retained by the %AcrobotGeometry
  /// system.  The return value pointer is an alias of the new
  /// %AcrobotGeometry system that is owned by the `builder`.
  static const AcrobotGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& acrobot_state_port,
      geometry::SceneGraph<double>* scene_graph) {
    return AddToBuilder(builder, acrobot_state_port, AcrobotParams<double>(),
                        scene_graph);
  }

 private:
  AcrobotGeometry(const AcrobotParams<double>& acrobot_params,
                  geometry::SceneGraph<double>*);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_{};
  // The frames for the two links.
  geometry::FrameId upper_link_frame_id_{};
  geometry::FrameId lower_link_frame_id_{};

  // Local copy of the parameter required for the forward kinematics (the
  // length of the upper link).
  const double l1_;
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
