#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace meshcat {

/** %MeshcatMouseSpring lets a user drag the bodies of a MultibodyPlant with the
mouse in a Meshcat browser, applying a virtual spring force that pulls the
grabbed point toward the cursor.

This system reads the drag state from Meshcat (see
geometry::Meshcat::GetVirtualSpringKinematics()) and outputs a corresponding
ExternallyAppliedSpatialForce on the dragged body. Connecting that
output to MultibodyPlant::get_applied_spatial_force_input_port() applies the
force; AddToBuilder() performs that connection along with the input connections.

@system
name: MeshcatMouseSpring
input_ports:
- body_poses
- body_spatial_velocities
output_ports:
- applied_spatial_force
@endsystem

The `body_poses` and `body_spatial_velocities` inputs come from the same-named
MultibodyPlant output ports.

With `m` the dragged body's mass, the applied force is `f = m⋅a`, where:
  - `a = stiffness * d - sqrt(stiffness) * v_anchor`,
  - `d` is the spring's displacement `target - anchor`, with its magnitude
    capped at `max_displacement` (see below),
  - `anchor` is the grabbed point on the body,
  - `target` is the cursor position, and
  - `v_anchor` is the world velocity of the grabbed point.
All spatial quantities are measured and expressed in the world frame.

Dragging in the UI can easily lead to massive values for |target - anchor|;
the browser goes on tracking the cursor when it leaves the viewport. So, the
spring component can get arbitrarily large (accelerating the body to extreme
speeds). These extreme speeds confound the contact solver, leading to tunneling
and NaNs.

We cap the spring's *displacement* instead of the overall acceleration, because
we can realize a zero acceleration at arbitrarily high speeds -- it just
requires the damping term to be equal in magnitude to the spring term. So, we'd
still get tunneling and NaNs. Limiting the spring's displacement limits the
impulse implied by the drag. And as it is capped, there is a natural equilibrium
for the drag term that ultimately limits the acceleration.

When no drag is in progress the output is empty. Any body with geometry
published to Meshcat by a geometry::MeshcatVisualizer can be dragged; the world
body cannot.

@ingroup visualization */
class MeshcatMouseSpring final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatMouseSpring);

  /** The default mass-normalized spring stiffness, in 1/s². */
  static constexpr double kDefaultStiffness = 100.0;

  /** The default cap on the spring's displacement, in m. At #kDefaultStiffness,
  it is invisible during ordinary dragging (where the body tracks the cursor
  closely) and engages only when the cursor is flung far from the manipulated
  body. */
  static constexpr double kDefaultMaxDisplacement = 1.0;

  /** Constructs a %MeshcatMouseSpring for the given `plant`.

  @param meshcat The Meshcat instance the user will interact with. The pointer
  is aliased and must outlive this system.

  @param plant The MultibodyPlant whose bodies can be dragged. The pointer is
  aliased and must outlive this system; the plant must already be finalized and
  registered as a geometry source with `scene_graph`.

  @param scene_graph The SceneGraph that `plant` is registered with. It is used
  at construction to read the frame names identifying draggable bodies in the
  Meshcat scene tree; the reference is used only during construction and is not
  retained.

  @param stiffness The mass-normalized spring stiffness, in 1/s²; see the class
  overview for the force it produces.

  @param max_displacement The cap on the spring's displacement magnitude, in
  m; see the class overview.

  @pre plant->is_finalized() is true.
  @pre plant is registered as a geometry source with scene_graph.
  @pre finite stiffness >= 0.
  @pre finite max_displacement > 0. */
  MeshcatMouseSpring(std::shared_ptr<geometry::Meshcat> meshcat,
                     const MultibodyPlant<double>* plant,
                     const geometry::SceneGraph<double>& scene_graph,
                     double stiffness = kDefaultStiffness,
                     double max_displacement = kDefaultMaxDisplacement);

  ~MeshcatMouseSpring() final;

  /** Returns the input port for the bodies' poses in the world frame (a
  `std::vector<math::RigidTransform<double>>`). */
  const systems::InputPort<double>& get_body_poses_input_port() const {
    return this->get_input_port(body_poses_input_port_);
  }

  /** Returns the input port for the bodies' spatial velocities in the world
  frame (a `std::vector<SpatialVelocity<double>>`). */
  const systems::InputPort<double>& get_body_spatial_velocities_input_port()
      const {
    return this->get_input_port(body_spatial_velocities_input_port_);
  }

  /** Returns the output port for the applied spatial forces (a
  `std::vector<ExternallyAppliedSpatialForce<double>>`). */
  const systems::OutputPort<double>& get_applied_spatial_force_output_port()
      const {
    return this->get_output_port(applied_spatial_force_output_port_);
  }

  /** Adds a %MeshcatMouseSpring to `builder` and connects it to `plant`'s
  body-pose and body-spatial-velocity output ports and its applied-spatial-force
  input port. Returns a reference to the newly-added system.

  @pre plant is part of builder and is finalized.
  @pre plant is registered as a geometry source with scene_graph.
  @pre `plant`'s applied-spatial-force input port is not already connected. */
  static MeshcatMouseSpring& AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const MultibodyPlant<double>* plant,
      const geometry::SceneGraph<double>& scene_graph,
      std::shared_ptr<geometry::Meshcat> meshcat,
      double stiffness = kDefaultStiffness,
      double max_displacement = kDefaultMaxDisplacement);

 private:
  friend class MeshcatMouseSpringTester;

  // Builds the map from each body's scoped Meshcat-path segment to its index.
  void BuildPathToBodyMap(const geometry::SceneGraph<double>& scene_graph);

  // Returns the spring force produced by the given drag state, or an empty
  // vector if there is no drag or the dragged path doesn't name a (non-world)
  // body of plant_. The poses X_WB_all and spatial velocities V_WB_all of all
  // the plant's bodies are indexed by BodyIndex.
  std::vector<ExternallyAppliedSpatialForce<double>>
  CalcAppliedSpatialForceFromDrag(
      const std::optional<geometry::Meshcat::VirtualSpringKinematics>& drag,
      const std::vector<math::RigidTransform<double>>& X_WB_all,
      const std::vector<SpatialVelocity<double>>& V_WB_all) const;

  void CalcAppliedSpatialForce(
      const systems::Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* forces) const;

  std::shared_ptr<geometry::Meshcat> meshcat_;
  const MultibodyPlant<double>& plant_;
  const double stiffness_;
  const double max_displacement_;

  // Maps each (non-world) body's scoped frame name as it appears in the Meshcat
  // scene tree (e.g. "my_model/my_body", or just "my_body" for the default
  // model instance) to that body's index. The leading "/drake/<prefix>/" and
  // any trailing geometry path are matched separately, so this is independent
  // of the visualizer prefix.
  std::map<std::string, BodyIndex> path_to_body_;

  systems::InputPortIndex body_poses_input_port_;
  systems::InputPortIndex body_spatial_velocities_input_port_;
  systems::OutputPortIndex applied_spatial_force_output_port_;
};

namespace internal {

/* Returns the index of the MultibodyPlant body whose stored Meshcat scene-tree
path segment matches `path`, or std::nullopt if none matches. `path_to_body`
maps each body's scoped path segment (e.g. "model/body", or just "body" for the
default and world model instances) to its index. Exposed for testing; see the
.cc for the exact matching rules. */
std::optional<BodyIndex> FindBodyForPath(
    const std::map<std::string, BodyIndex>& path_to_body,
    const std::string& path);

}  // namespace internal

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
