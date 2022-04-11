#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/** A System that connects to the MultibodyPlant in order to model the
simplified dynamics of an airfoil (or hydrofoil).  Currently it only supports
flat-plate aerodynamics, but this can be extended in the future.

@system
name: Wing
input_ports:
- body_poses
- body_spatial_velocities
- wind_velocity_at_aerodynamic_center (optional)
- fluid_density (optional)
output_ports:
- spatial_force
- aerodynamic_center
@endsystem

- The optional wind velocity input is a three-element BasicVector<T>
  representing the translational velocity of the wind in world coordinates at
  the aerodynamic center relative to the world origin.  See
  get_aerodynamic_center_output_port() for more details.
- It is expected that the body_poses input should be connected to the
  @ref MultibodyPlant::get_body_poses_output_port() "MultibodyPlant body_poses
  output port" and that body_spatial_velocities input should be connected to
  the @ref MultibodyPlant::get_body_spatial_velocities_output_port()
  "MultibodyPlant body_spatial_velocities output port"
- The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
  expected that this output will be connected to the @ref
  MultibodyPlant::get_applied_spatial_force_input_port()
  "externally_applied_spatial_force input port" of the MultibodyPlant.

@tparam_default_scalar
@ingroup multibody_systems
*/
template <typename T>
class Wing final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Wing);

  /** Constructs a system describing a single wing using flat-plate
   aerodynamics.
   @param body_index indicates the body on which the aerodynamic forces are
   applied.
   @param surface_area is the total surface area of the wing in meters squared.
   @param X_BodyWing is the pose of wing frame relative to the body frame,
   whose origin is at the aerodynamic center of the wing, the positive x-axis
   points along the chord towards nominal direction of travel (e.g. towards the
   nose of the plane), the positive y-axis points along the span, and the
   z-axis points up. According to thin airfoil theory, the aerodynamic center
   of a symmetric wing (like this flat plate), should be located at the
   quarter-chord position.
   @param fluid_density is the density of the fluid in kg/m^3. The default
   value is the density of dry air at 20 deg Celsius at sea-level. This value
   is only used if the optional fluid_density input port is not connected.
  */
  Wing(const BodyIndex& body_index, double surface_area,
       const math::RigidTransform<double>& X_BodyWing =
           math::RigidTransform<double>::Identity(),
       double fluid_density = 1.204);

  /** Scalar-converting copy constructor.  See @ref system_scalar_conversion. */
  template <typename U>
  explicit Wing(const Wing<U>& other)
      : Wing<T>(other.body_index_, other.surface_area_, other.X_BodyWing_,
                other.default_fluid_density_) {}

  /** Returns a reference to the body_poses input port.  It is anticipated
  that this port will be connected the body_poses output port of a
  MultibodyPlant. */
  const systems::InputPort<T>& get_body_poses_input_port() const {
    return this->get_input_port(0);
  }

  /** Returns a reference to the body_spatial_velocities input port.  It is
  anticipated that this port will be connected the body_spatial_velocities
  output port of a MultibodyPlant. */
  const systems::InputPort<T>& get_body_spatial_velocities_input_port() const {
    return this->get_input_port(1);
  }

  /** Returns a reference to the input port for the optional three-element
  BasicVector<T> representing the translational velocity of the wind in world
  coordinates at the aerodynamic center relative to the world origin. If this
  port is not connected, then the wind speed is taken to be zero. */
  const systems::InputPort<T>& get_wind_velocity_input_port() const {
    return this->get_input_port(2);
  }

  /** Returns a reference to the optional fluid_density input port, which
   accepts a scalar vector in units kg/m^3. This port is provided to support
   vehicles which must take into account variations in atmospheric density;
   such as a spacecraft during re-entry.  If left unconnected, the aerodynamic
   forces will be calculated using the default fluid density passed in the
   constructor. */
  const systems::InputPort<T>& get_fluid_density_input_port() const {
    return this->get_input_port(3);
  }

  /** Returns a reference to the spatial_forces output port.  It is anticipated
  that this port will be connected to the @ref
  MultibodyPlant::get_applied_spatial_force_input_port() "applied_spatial_force"
  input port of a MultibodyPlant. */
  const systems::OutputPort<T>& get_spatial_force_output_port() const {
    return this->get_output_port(0);
  }

  /** Returns a 3-element position of the aerodynamic center of the wing in
   world coordinates. This output port does not depend on the optional wind
   velocity input port, so it may be used to compute the wind velocity at the
   aerodynamic center without causing any algebraic loops in the Diagram. For
   instance, the following (sub-)Diagram could be used to implement a wind
   field:
                         ┌────────────┐
                      ┌──┤ Wind Field │◄─┐
                      │  └────────────┘  │
                      │   ┌──────────┐   │
                      └──►│   Wing   ├───┘
         wind_velocity_at_└──────────┘aerodynamic_center
         aerodynamic_center
   */
  const systems::OutputPort<T>& get_aerodynamic_center_output_port() const {
    return this->get_output_port(1);
  }

  /** Helper method that constructs a Wing and connects the input and output
   ports to the MultibodyPlant.

   @param builder is a DiagramBuilder that the Wing will be added to.
   @param plant is the MultibodyPlant containing the body referenced by
   `body_index`, which the wing ports will be connected to.

   See the Wing constructor for details on the remaining parameters.
   */
  static Wing<T>* AddToBuilder(systems::DiagramBuilder<T>* builder,
                               const multibody::MultibodyPlant<T>* plant,
                               const BodyIndex& body_index, double surface_area,
                               const math::RigidTransform<double>& X_BodyWing =
                                   math::RigidTransform<double>::Identity(),
                               double fluid_density = 1.204);

 private:
  // Calculates the spatial forces in the world frame as expected by the
  // applied_spatial_force input port of MultibodyPlant.
  void CalcSpatialForce(
      const systems::Context<T>& context,
      std::vector<ExternallyAppliedSpatialForce<T>>* spatial_force) const;

  // Calculates the aerodynamic center output port.
  void CalcAerodynamicCenter(const systems::Context<T>& context,
                             systems::BasicVector<T>* aerodynamic_center) const;

  // Declare friendship to enable scalar conversion.
  template <typename U>
  friend class Wing;

  const BodyIndex body_index_;
  const math::RigidTransform<double> X_BodyWing_;
  const double surface_area_;
  const double default_fluid_density_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Wing)
