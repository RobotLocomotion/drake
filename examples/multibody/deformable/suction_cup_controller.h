#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace deformable {

/* We create a leaf system that outputs the desire state of a suction cup
 gripper mounted on a prismatic joint to the world. This control is a time-based
 state machine, where the desired state changes based on the context time. The
 desired state is described through two output ports, one containing the desired
 position and velocity of the prismatic joint to the world, the other containing
 the maximum suction force density. The suction cup is first lowered to approach
 the object, then the suction force is turned on to pick up the object. Then the
 suction cup is raised to the initial height and the suction force is turned off
 to release the object. This is strictly for demo purposes and is not intended
 as a generalized model for suction grippers. */
class SuctionCupController : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuctionCupController);

  /* Constructs a SuctionCupController system with the given parameters. The
   trajectory is characterized by 4 key time stamps and 2 key positions. The 4
   key time stamps are `approach_time`, `start_suction_time`,
   `retrieve_time`, and `release_suction_time`. They must be in increasing
   order. The 2 key positions are `initial_height` and `object_height`.
   @param[in] initial_height        Initial height of the gripper.
   @param[in] object_height         The estimated height of the object. We turn
                                    suction on when the gripper first reach this
                                    height.
   @param[in] approach_time         The time at which the gripper starts to
                                    approach the manipuland.
   @param[in] start_suction_time    The time at which suction force turns on.
   @param[in] retrieve_time         The time at which to start lifting the
                                    gripper.
   @param[in] release_suction_time  The time at which suction turns off. */
  SuctionCupController(double initial_height, double object_height,
                       double approach_time, double start_suction_time,
                       double retrieve_time, double release_suction_time);

  const systems::OutputPort<double>& desired_state_output_port() const {
    return get_output_port(desired_state_port_index_);
  }

  const systems::OutputPort<double>& maximum_force_density_port() const {
    return get_output_port(maximum_force_density_port_index_);
  }

 private:
  /* Computes the output desired state of the suction cup. */
  void CalcDesiredState(const systems::Context<double>& context,
                        systems::BasicVector<double>* desired_state) const;

  /* Computes the maximum suction force in Newtons based on time. */
  void CalcMaxForceDensity(
      const systems::Context<double>& context,
      systems::BasicVector<double>* max_force_density) const;

  double initial_height_{};
  double object_height_{};
  double approach_time_{};
  double start_suction_time_{};
  double retrieve_time_{};
  double release_suction_time_{};

  int desired_state_port_index_{};
  int maximum_force_density_port_index_{};
};

}  // namespace deformable
}  // namespace examples
}  // namespace drake
