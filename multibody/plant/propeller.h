#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/tree/body.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/** Parameters that describe the kinematic frame and force-production properties
of a single propeller. */
struct PropellerInfo {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PropellerInfo);

  explicit PropellerInfo(const BodyIndex& body_index_,
                         const math::RigidTransform<double>& X_BP_ =
                             math::RigidTransform<double>::Identity(),
                         double thrust_ratio_ = 1.0, double moment_ratio_ = 0.0)
      : body_index(body_index_),
        X_BP(X_BP_),
        thrust_ratio(thrust_ratio_),
        moment_ratio(moment_ratio_) {}

  /** The BodyIndex of a Body in the MultibodyPlant to which the propeller is
  attached.  The spatial forces will be applied to this body. */
  BodyIndex body_index;

  /** Pose of the propeller frame P measured in the body frame B. @default is
  the identity matrix. */
  math::RigidTransform<double> X_BP{};

  /** The z component (in frame P) of the spatial force will have magnitude
  `thrust_ratio*command` in Newtons. The default is 1 (command in Newtons), but
  this can also be used to scale an actuator command to the resulting force. */
  double thrust_ratio{1.0};

  /** The moment about the z axis (in frame P) of the spatial force will have
  magnitude `moment_ratio*command` in Newton-meters. The default is 0, which
  makes the propeller a simple Cartesian force generator. */
  double moment_ratio{0.0};
};

/** A System that connects to the MultibodyPlant in order to model the effects
of one or more controlled propellers acting on a Body.

@system
name: Propeller
input_ports:
- command
- body_poses
output_ports:
- spatial_forces
@endsystem

- The command input is a BasicVector<T> with one element per propeller.
- It is expected that the body_poses input should be connected to the
  @ref MultibodyPlant::get_body_poses_output_port() "MultibodyPlant body_poses
  output port".
- The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
  expected that this output will be connected to the @ref
  MultibodyPlant::get_applied_spatial_force_input_port()
  "externally_applied_spatial_force input port" of the MultibodyPlant.
- This system does not have any state.

The resulting iᵗʰ spatial force will have a force component in the z-axis of
the iᵗʰ propeller frame with magnitude `thrust_ratio * command` Newtons, and a
moment around the z-axis with magnitude `moment_ratio * command` Newton-meters.
(Including these moments tends to be important -- a quadrotor does not have a
stabilizable linearization around a hovering fixed point in 3D without them).

@note Set PropellerInfo::moment_ratio to zero if you want a simple thruster
which applies only a force (no moment) in the Propeller coordinates.

@tparam_default_scalar
@ingroup multibody_systems
*/
template <typename T>
class Propeller final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Propeller);

  /** Constructs a system describing a single propeller.
  @see PropellerInfo for details on the arguments. */
  Propeller(const BodyIndex& body_index,
            const math::RigidTransform<double>& X_BP =
                math::RigidTransform<double>::Identity(),
            double thrust_ratio = 1.0, double moment_ratio = 0.0);

  /** Constructs a system describing multiple propellers.
  @see PropellerInfo. */
  explicit Propeller(const std::vector<PropellerInfo>& propeller_info);

  /** Scalar-converting copy constructor.  See @ref system_scalar_conversion. */
  template <typename U>
  explicit Propeller(const Propeller<U>& other)
      : Propeller<T>(std::vector<PropellerInfo>(other.info_.begin(),
                                                other.info_.end())) {}

  /** Returns the number of propellers modeled by this system. */
  int num_propellers() const { return info_.size(); }

  /** Returns a reference to the vector-valued input port for the propeller
  commands.  It has size `num_propellers()`. */
  const systems::InputPort<T>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  /** Returns a reference to the body_poses input port.  It is anticipated
  that this port will be connected the body_poses output port of a
  MultibodyPlant. */
  const systems::InputPort<T>& get_body_poses_input_port() const {
    return this->get_input_port(1);
  }

  /** Returns a reference to the spatial_forces output port.  It is anticipated
  that this port will be connected to the @ref
  MultibodyPlant::get_applied_spatial_force_input_port() "applied_spatial_force"
  input port of a MultibodyPlant. */
  const systems::OutputPort<T>& get_spatial_forces_output_port() const {
    return this->get_output_port(0);
  }

 private:
  // Calculates the spatial forces in the world frame as expected by the
  // applied_spatial_force input port of MultibodyPlant.
  void CalcSpatialForces(
      const systems::Context<T>& context,
      std::vector<ExternallyAppliedSpatialForce<T>>* spatial_forces) const;

  std::vector<PropellerInfo> info_;

  // Declare friendship to enable scalar conversion.
  template <typename U>
  friend class Propeller;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Propeller)
