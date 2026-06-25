#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace meshcat {

/** %MeshcatMouseSpring lets a user drag the bodies of a MultibodyPlant with the
mouse in a Meshcat browser: holding Ctrl and dragging a body with the left mouse
button applies a virtual spring force that pulls the grabbed point toward the
cursor.

This system reads the drag state from Meshcat (see
geometry::Meshcat::GetObjectDrag()) and outputs a corresponding
geometry::ExternallyAppliedSpatialForce on the dragged body. Connecting that
output to MultibodyPlant::get_applied_spatial_force_input_port() applies the
force; AddToBuilder() performs that connection along with the input connections.

@system
name: MeshcatMouseSpring
input_ports:
- body_poses
- body_spatial_velocities
output_ports:
- spatial_forces
@endsystem

The `body_poses` and `body_spatial_velocities` inputs come from the same-named
MultibodyPlant output ports.

With `m` the dragged body's mass, the applied force (in the world frame) is
`m * stiffness * (target - anchor) - m * sqrt(stiffness) * v_anchor`, where
`anchor` is the grabbed point on the body, `target` is the cursor position, and
`v_anchor` is the world velocity of the grabbed point. Scaling by `m` makes the
translational response frequency `sqrt(stiffness)` and damping ratio independent
of the body's mass.

When no drag is in progress the output is empty. Any body with geometry
published to Meshcat by a geometry::MeshcatVisualizer can be dragged; the world
body cannot.

This system is `double`-only, because Meshcat reports drag state as plain
doubles and mouse interaction is not meaningful for other scalar types.

@ingroup visualization */
class MeshcatMouseSpring final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatMouseSpring);

  /** The default mass-normalized spring stiffness, in 1/s². */
  static constexpr double kDefaultStiffness = 1000.0;

  /** Constructs a %MeshcatMouseSpring for the given `plant`.

  @param meshcat The Meshcat instance the user will interact with. The pointer
  is aliased and must outlive this system.

  @param plant The MultibodyPlant whose bodies can be dragged. The pointer is
  aliased and must outlive this system; the plant must already be finalized.

  @param stiffness The mass-normalized spring stiffness, in 1/s²; see the class
  overview for the force it produces.

  @pre plant->is_finalized() is true.
  @pre stiffness >= 0. */
  MeshcatMouseSpring(std::shared_ptr<geometry::Meshcat> meshcat,
                     const MultibodyPlant<double>* plant,
                     double stiffness = kDefaultStiffness);

  ~MeshcatMouseSpring() final;

  /** Returns the input port for the bodies' poses (a
  `std::vector<math::RigidTransform<double>>`). */
  const systems::InputPort<double>& get_body_poses_input_port() const {
    return this->get_input_port(body_poses_input_port_);
  }

  /** Returns the input port for the bodies' spatial velocities (a
  `std::vector<SpatialVelocity<double>>`). */
  const systems::InputPort<double>& get_body_spatial_velocities_input_port()
      const {
    return this->get_input_port(body_spatial_velocities_input_port_);
  }

  /** Returns the output port for the applied spatial forces (a
  `std::vector<ExternallyAppliedSpatialForce<double>>`). */
  const systems::OutputPort<double>& get_spatial_forces_output_port() const {
    return this->get_output_port(spatial_forces_output_port_);
  }

  /** Adds a %MeshcatMouseSpring to `builder` and connects it to `plant`'s
  body-pose and body-spatial-velocity output ports and its applied-spatial-force
  input port. Returns a reference to the newly-added system.

  @pre plant is part of builder and is finalized.
  @pre `plant`'s applied-spatial-force input port is not already connected. */
  static MeshcatMouseSpring& AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const MultibodyPlant<double>* plant,
      std::shared_ptr<geometry::Meshcat> meshcat,
      double stiffness = kDefaultStiffness);

 private:
  // Builds the map from each body's scoped frame name to its index.
  void BuildPathToBodyMap(const MultibodyPlant<double>& plant);

  void CalcSpatialForces(
      const systems::Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* forces) const;

  std::shared_ptr<geometry::Meshcat> meshcat_;
  const MultibodyPlant<double>* const plant_;
  const double stiffness_;

  // Maps each (non-world) body's scoped frame name as it appears in the Meshcat
  // scene tree (e.g. "my_model/my_body", or just "my_body" for the default
  // model instance) to that body's index. The leading "/drake/<prefix>/" and
  // any trailing geometry path are matched separately, so this is independent
  // of the visualizer prefix.
  std::map<std::string, BodyIndex> path_to_body_;

  systems::InputPortIndex body_poses_input_port_;
  systems::InputPortIndex body_spatial_velocities_input_port_;
  systems::OutputPortIndex spatial_forces_output_port_;
};

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
