#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/box/box_plant.h"

namespace drake {
namespace examples {
namespace box {

/// Expresses a BoxPlants's geometry to a SceneGraph.
///
/// @system{BoxGeometry,
///    @input_port{state},
///    @output_port{geometry_pose}
/// }
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
template <typename T>
class BoxGeometryTemplate final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BoxGeometryTemplate);
  ~BoxGeometryTemplate() final;

  /// Creates, adds, and connects a BoxGeometry system into the given
  /// `builder`.  Both the `box_state.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// The `scene_graph` pointer is not retained by the %BoxGeometry system.
  /// The return value pointer is an alias of the new %BoxGeometry system
  /// that is owned by the `builder`.
  static const BoxGeometryTemplate<T>* AddToBuilder(
      systems::DiagramBuilder<T>* builder,
       const BoxPlant<T>& box, 
      geometry::SceneGraph<T>* scene_graph, std::string srcName)
      {
        return AddToBuilder(builder, box, box.get_state_output_port(), scene_graph, srcName);
      }

  static const BoxGeometryTemplate<T>* AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const BoxPlant<T>& box, 
      const systems::OutputPort<T>& box_state_output_port,
      geometry::SceneGraph<T>* scene_graph, std::string srcName);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit BoxGeometryTemplate(const BoxGeometryTemplate<U>& other) ;
 private:
 template <typename>
 friend class BoxGeometryTemplate;
 
  explicit BoxGeometryTemplate(geometry::SceneGraph<T>*, const BoxPlant<T>& box, std::string srcName);
  void OutputGeometryPose(const systems::Context<T>&,
                          geometry::FramePoseVector<T>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_;
  // The id for the box (arm + point mass) frame.
  geometry::FrameId frame_id_;
  geometry::SceneGraph<T>* scene_graph_;
};

typedef BoxGeometryTemplate<double> BoxGeometry;
}  // namespace box
}  // namespace examples
}  // namespace drake
