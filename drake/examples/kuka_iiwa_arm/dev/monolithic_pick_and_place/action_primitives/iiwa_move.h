#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitive_base.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/// An `ActionPrimitive` for commanding the joint angles of the Kuka IIWA Arm.
class IiwaMove : public ActionPrimitive {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaMove)

  IiwaMove(const RigidBodyTree<double>& iiwa,
           const double desired_update_interval);

  /// This method gets the abstract input port corresponding to the
  /// `IiwaActionInput`.
  const systems::InputPortDescriptor<double>& get_primitive_input_port() const {
    return this->get_input_port(input_port_primitive_input_);
  }

  /// This method gets the abstract output port corresponding to the
  /// the `robotlocomotion::robot_plan_t`.
  const systems::OutputPort<double>& get_robot_plan_output_port()
      const {
    return this->get_output_port(output_port_plan_);
  }

 protected:
  std::vector<std::unique_ptr<systems::AbstractValue>>
  AllocateExtendedAbstractState() const final;

  void SetExtendedDefaultState(const systems::Context<double>& context,
                               systems::State<double>* state) const override;


  void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const final;

 private:
  // InternalState relevant only to this primitive.
  struct InternalState;

  void OutputRobotPlan(
      const systems::Context<double>& context,
      robotlocomotion::robot_plan_t* robot_plan_output) const;

  const unsigned int internal_state_index_{0};
  const int input_port_primitive_input_{-1};
  const int output_port_plan_{-1};
  const RigidBodyTreed& iiwa_tree_;
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
