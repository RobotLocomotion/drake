#pragma once
#include <vector>

#include "drake/examples/mass_spring_cloth/cloth_spring_model.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace mass_spring_cloth {

/** Expresses a ClothSpringModel's visualization geometry to a SceneGraph. This
 visualization takes the particle_positions output from ClothSpringModel.

 @system
 name: ClothSpringModelGeometry
 input_ports:
 - particle_positions
 output_ports:
 - geometry_pose
 @endsystem

 The visualization shows the particles as spheres without any visualization of
 the springs.

 This class has no public constructor; instead use the AddToBuilder() static
 method to create and add it to a DiagramBuilder directly.
 */
class ClothSpringModelGeometry final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ClothSpringModelGeometry);

  /** Creates, adds, and connects a ClothSpringModelGeometry system into the
   given `builder`.  Both the `cloth_spring_model` and `scene_graph` systems
   must have been added to the given `builder` already.

   The `scene_graph` pointer is not retained by the %ClothSpringModelGeometry
   system. The return value pointer is an alias of the new
   %ClothSpringModelGeometry system that is owned by the `builder`.

   @throws std::exception if @p cloth_spring_model or @p scene_graph is not
   already added to the given @p builder.
   */
  static const ClothSpringModelGeometry& AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const ClothSpringModel<double>& cloth_spring_model,
      geometry::SceneGraph<double>* scene_graph);

 private:
  ClothSpringModelGeometry(geometry::SceneGraph<double>* scene_graph,
                           int num_particles, double h);
  void OutputGeometryPose(const systems::Context<double>&,
                          geometry::FramePoseVector<double>*) const;

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_{};
  // The identifier for each particle's frame.
  std::vector<geometry::FrameId> frame_ids_;
  // Number of particles.
  int num_particles_{};
  double particle_radius_{};
};

}  // namespace mass_spring_cloth
}  // namespace examples
}  // namespace drake
