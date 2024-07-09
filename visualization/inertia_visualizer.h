#pragma once

#include <utility>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

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

@warning The visualized inertia shows the inertia of the `plant` bodies when
this system was constructed. If you edit the plant's inertia values after
construction (e.g., by changing a body's mass), the visualization will not be
updated.

@tparam_default_scalar
@ingroup visualization */
template <typename T>
class InertiaVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InertiaVisualizer);

  /** Creates an instance of %InertiaVisualizer.
  The plant must be finalized. */
  InertiaVisualizer(const multibody::MultibodyPlant<T>& plant,
                    geometry::SceneGraph<T>* scene_graph);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit InertiaVisualizer(const InertiaVisualizer<U>& other);

  ~InertiaVisualizer() final;

  /** Adds a new InertiaVisualizer to the given `builder` and connects it to the
  given `plant` and `scene_graph`. */
  static const InertiaVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const multibody::MultibodyPlant<T>& plant,
      geometry::SceneGraph<T>* scene_graph);

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

  geometry::SourceId source_id_;
  std::vector<Item> items_;
};

namespace internal {
/* Returns the inertia geometry Ellipsoid and the X_BoBcm that represents the
given body's moments of inertia. */
template <typename T>
std::pair<geometry::Ellipsoid, math::RigidTransform<double>>
CalculateInertiaGeometry(const multibody::RigidBody<T>& body,
                         const systems::Context<T>& plant_context);
}  // namespace internal

}  // namespace visualization
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::InertiaVisualizer);
