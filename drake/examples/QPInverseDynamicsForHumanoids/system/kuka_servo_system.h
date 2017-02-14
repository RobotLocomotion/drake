#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/discrete_time_plan_eval_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends DiscreteTimePlanEvalSystem. It generates QpInput to track
 * desired instantaneous position, velocity and acceleration.
 */
class KukaServoSystem : public DiscreteTimePlanEvalSystem {
 public:
  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Time step
   */
  KukaServoSystem(const RigidBodyTree<double>& robot,
                  const std::string& alias_groups_file_name,
                  const std::string& param_file_name, double dt);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  /**
   * Initializes the internal states.
   */
  void Initialize(systems::State<double>* state);

  /**
   * Initializes the internal states.
   */
  void Initialize(systems::Context<double>* context) {
    systems::State<double>* servo_state = context->get_mutable_state();
    Initialize(servo_state);
  }

  inline const systems::InputPortDescriptor<double>&
  get_input_port_desired_state_and_acceleration() const {
    return get_input_port(input_port_index_desired_state_and_acceleration_);
  }

 private:
  int input_port_index_desired_state_and_acceleration_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
