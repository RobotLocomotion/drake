#pragma once

#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/visualization/inertia_visualizer_params.h"

namespace drake {
namespace visualization {

/** InertiaVisualizer provides illustration geometry to reflect the equivalent
inertia of all bodies in a MultibodyPlant that are not welded to the world.

Instead of constructing this system directly, most users should use
AddDefaultVisualization() which automatically uses this system.

 @system
 name: InertiaVisualizer
 input_ports:
 - plant_geometry_pose
 output_ports:
 - geometry_pose
 @endsystem

@tparam_default_scalar
@ingroup visualization */
template <typename T>
class InertiaVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InertiaVisualizer)

  /** Creates an instance of %InertiaVisualizer.
  The plant must be finalized. */
  InertiaVisualizer(const multibody::MultibodyPlant<T>& plant,
                    geometry::SceneGraph<T>* scene_graph,
                    InertiaVisualizerParams params = {});

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit InertiaVisualizer(const InertiaVisualizer<U>& other);

  ~InertiaVisualizer() final;

  /** Adds a new InertiaVisualizer to the given `builder` and connects it to the
  given `plant` and `scene_graph`, using the settings in `params`. */
  static const InertiaVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const multibody::MultibodyPlant<T>& plant,
      geometry::SceneGraph<T>* scene_graph,
      InertiaVisualizerParams params = {});

  /** (Advanced) Returns the source_id for our visualization geometries.
  Most users will not need to use this. */
  geometry::SourceId source_id() const { return source_id_; }

 private:
  template <typename>
  friend class InertiaVisualizer;

  /* Each piece of visualization geometry has an Item to track its associated
  indices and BoBcm transform. */
  struct Item {
    multibody::BodyIndex body;
    geometry::FrameId Bo_frame;
    geometry::FrameId Bcm_frame;
    geometry::GeometryId geometry;
    math::RigidTransform<double> X_BBcm;
  };

  /* Updates `this->items_` to match the inertia values in the given context. */
  void UpdateItems(const multibody::MultibodyPlant<T>& plant,
                   const systems::Context<T>& plant_context,
                   geometry::SceneGraph<T>* scene_graph);

  /* Calculates the "geometry_pose" output port. */
  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  const InertiaVisualizerParams params_;
  geometry::SourceId source_id_;
  std::vector<Item> items_;
};

}  // namespace visualization
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::InertiaVisualizer)
