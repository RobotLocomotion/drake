#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace deformable_torus {

/* We create a leaf system that outputs the desire state of a suction cup
 gripper mounted on a prismatic joint to the world. The desired state is
 described through two output ports, one containing the desired position and
 velocity of the prismatic joint to the world, the other containing the on/off
 state of the suction force. The suction cup is first lowered to approach the
 object, then the suction force is turned on to pick up the object. Then the
 suction cup is raised to the intial height and the suction force is turned off
 to release the object. This is strictly for demo purposes and is not intended
 as a generalized model for suction grippers. */
class SuctionCupController : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuctionCupController);

  /* Constructs a SuctionCupController system with the given parameters.
   @param[in] initial_height   Initial height of the gripper.
   @param[in] pick_height      The height at suction turns on.
   @param[in] pick_time        The time at which to start suction.
   @param[in] travel_time      The time it takes for the gripper to travel from
                               the initial height to the pick height.
   @param[in] lift_time        The time at which to start raising the gripper.
   @param[in] drop_time        The time at which to turn off the suction. */
  SuctionCupController(double initial_height, double pick_height,
                       double pick_time, double travel_time, double lift_time,
                       double drop_time);

  const systems::OutputPort<double>& desired_state_output_port() const {
    return get_output_port(desired_state_port_index_);
  }

  const systems::OutputPort<double>& signal_output_port() const {
    return get_output_port(signal_port_index_);
  }

 private:
  /* Computes the output desired state of the suction cup. */
  void CalcDesiredState(const systems::Context<double>& context,
                        systems::BasicVector<double>* desired_state) const;

  /* Computes the on/off signal of the suction force based on time. */
  void CalcSignal(const systems::Context<double>& context,
                  systems::BasicVector<double>* signal) const;

  double initial_height_{};
  double pick_height_{};
  double pick_time_{};
  double travel_time_{};
  double lift_time_{};
  double drop_time_{};

  int desired_state_port_index_{};
  int signal_port_index_{};
};

}  // namespace deformable_torus
}  // namespace examples
}  // namespace drake
