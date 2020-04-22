#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/**
 Parameters that describe the kinematic frame and force-production properties
  of a single propeller.
  */
struct PropellerInfo {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PropellerInfo);

  /**
   Constructs and populates a PropellerInfo struct.
    @pre `multibody_plant` must have been registered with a `SceneGraph`.  (The
        `SceneGraph` does not actually have to exist in the diagram).
  */
  template <typename T>
  PropellerInfo(const MultibodyPlant<T>& multibody_plant, const Body<T>& body,
                const math::RigidTransform<double>& X_BP_,
                double thrust_ratio_ = 1.0, double moment_ratio_ = 0.0)
      : body_index(body.index()),
        frame_id(multibody_plant.GetBodyFrameIdOrThrow(body_index)),
        X_BP(X_BP_),
        thrust_ratio(thrust_ratio_),
        moment_ratio(moment_ratio_) {}

  /**
   The BodyIndex of a Body in the MultibodyPlant to which the propellor is
    attached.  The spatial forces will be applied to this body.
    */
  BodyIndex body_index;

  /**
   Identifier so that I can use the geometry_poses port of MultibodyPlant.
    */
  geometry::FrameId frame_id;

  /**
   A transform that describes the frame of the propeller, F_P, in the frame of
    the body, F_B.
    */
  math::RigidTransform<double> X_BP;

  /**
   The z component (in frame P) of the spatial force will have magnitude
    `thrust_ratio*command`.
    */
  double thrust_ratio;

  /**
   The moment about the z axis (in frame F_P) of the spatial force will have
    magnitude `moment_ratio*command`.
    */
  double moment_ratio;
};

/**
  A System that connects to the MultibodyPlant in order to model the effects of
  one or more controlled propellers acting on a Body.

  @system{Propeller,
    @input_port{command}
    @input_port{geometry_poses},
    @output_port{spatial_forces}
  }

  - The command input is a BasicVector<T> with one element per propeller.
  - It is expected that the geometry_pose input should be connected to the
    `geometry_poses` output of the MultibodyPlant.
  - The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
    expected that this output will be connected to the
    externally_applied_spatial_force input port of the MultibodyPlant.
  - This system does not have any state.

  The resulting ith spatial force will have a linear component in the z-axis of
  the ith propeller frame with magnitude `thrust_ratio * command` Newtons, and a
  moment around the z-axis with magnitude `moment_ratio * command`
  Newton-meters. (Including these moments tends to be important -- a quadrotor
  does not have a stabilizable linearization around a hovering fixed point in 3D
  without them).

  Note: Set moment_ratio to zero if you want a simple thruster which applies
  only a force (no moment) in the Propeller coordinates.

  Note: Due to the current lack of a better design pattern that supports scalar
  conversion (#13131), propeller forces can currently only be applied to
  aMultibodyPlant that registers with a SceneGraph.  The SceneGraph need not
  appear in the diagram with the Propeller; registering it with a SceneGraph is
  sufficient for the bodies' kinematic information to become available on the
  MultibodyPlant's geometry_poses output port.

  @tparam_default_scalar
  @ingroup multibody
*/
template <typename T>
class Propeller final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Propeller);

  /**
   Constructs a system describing a single propeller.
   @see PropellerInfo for details on the remaining arguments.
   */
  Propeller(const MultibodyPlant<T>& multibody_plant, const Body<T>& body,
            const math::RigidTransform<double>& X_BP, double thrust_ratio = 1.0,
            double moment_ratio = 0.0);

  /**
   Constructs a system describing multiple propellers.
   @see PropellerInfo.
   */
  explicit Propeller(const std::vector<PropellerInfo>& propeller_info);

  /** Scalar-converting copy constructor.  See @ref system_scalar_conversion. */
  template <typename U>
  explicit Propeller(const Propeller<U>& other)
      : Propeller<T>(std::vector<PropellerInfo>(other.info_.begin(),
                                                other.info_.end())) {}

  /**
   Returns the number of propellers modeled by this system.
   */
  int num_propellers() const { return info_.size(); }

  /**
   Returns a reference to the vector-valued input port for the propellor
   commands.  It has size `num_propellers()`.
   */
  const systems::InputPort<T>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  /**
   Returns a reference to the geometry_poses input port.  It is anticipated that
   this port will be connected the geometry_poses output port of a
   MultibodyPlant.
  */
  const systems::InputPort<T>& get_geometry_poses_input_port() const {
    return this->get_input_port(1);
  }

  /**
   Returns a reference to the external_forces output port.  It is anticipated
   that this port will be connected to the applied_spatial_force input port of a
   MultibodyPlant.
   */
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
