#pragma once

#include <memory>
#include <string>

#include "bot_core/robot_state_t.hpp"
#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/synchronous_world_state.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/**
 * A class that implements the Finite-State-Machine logic for the
 * Pick-And-Place demo. This system is to be used by coupling the outputs with
 * the `IiwaMove` and `GripperAction` `ActionPrimitive` systems and the inputs
 * are
 * to be supplied from `IiwaStatusSender`, `SchunkWsgStatusSender` and
 * `OracularStateEstimator` systems.
 */
class PickAndPlaceStateMachineSystem : public systems::LeafSystem<double> {
 public:
  /**
   * Constructor for the PickAndPlaceStateMachineSystem
   * @param iiwa_base, The pose of the base of the IIWA robot system.
   * @param update_interval : The update interval of the unrestricted update of
   * this system. This should be bigger than that of the PlanSource components.
   */
  PickAndPlaceStateMachineSystem(const Isometry3<double>& iiwa_base,
                                 const double update_interval = 0.1);

  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;

  /**
   * Allocates abstract value types for output @p descriptor. This function
   * allocates QpInput when @p matches the port for QpInput, and calls
   * ExtendedAllocateOutputAbstract() to allocate all the other derived class'
   * custom output types.
   */
  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const final;

  // This kind of a system is not a direct feedthrough.
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port, int output_port) const final {
    return false;
  }

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  const systems::InputPortDescriptor<double>& get_input_port_iiwa_state()
      const {
    return this->get_input_port(input_port_iiwa_state_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_box_state() const {
    return this->get_input_port(input_port_box_state_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_wsg_status()
      const {
    return this->get_input_port(input_port_wsg_status_);
  }

  const systems::InputPortDescriptor<double>&
  get_input_port_iiwa_action_status() const {
    return this->get_input_port(input_port_iiwa_action_status_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_wsg_action_status()
      const {
    return this->get_input_port(input_port_wsg_action_status_);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_iiwa_action()
      const {
    return this->get_output_port(output_port_iiwa_action_);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_wsg_action()
      const {
    return this->get_output_port(output_port_wsg_action_);
  }

 private:
  struct InternalState;

  RigidBodyTree<double> iiwa_tree_{};
  // Input ports.
  int input_port_iiwa_state_{-1};
  int input_port_box_state_{-1};
  int input_port_wsg_status_{-1};
  int input_port_iiwa_action_status_{-1};
  int input_port_wsg_action_status_{-1};
  // Output ports.
  int output_port_iiwa_action_{-1};
  int output_port_wsg_action_{-1};

  const Isometry3<double> iiwa_base_;

  const std::unique_ptr<IiwaIkPlanner> planner_{nullptr};
  const std::unique_ptr<SynchronousWorldState> world_state_{nullptr};
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
