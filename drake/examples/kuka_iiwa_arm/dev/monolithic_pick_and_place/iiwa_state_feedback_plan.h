#pragma once

#include <memory>
#include <string>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/// This class implements a source of joint positions for an iiwa arm.
/// It has two input ports, one for robot_plan_t messages containing a
/// plan to follow, and another vector valued port for directly providing
/// state feedback.
///
/// If no plan has been received, the system will create an initial
/// plan on the first unrestricted state update which commands the arm
/// to hold at the measured position. This system will also function in
/// the same manner if a plan is received with no states.
///
/// This class is largely similar to IiwaPlanSource in its functionality
/// but can be hooked up by direct state feedback and avoids a direct
/// feedthrough thus eliminating algebriac loops when used in conjunction
/// with a RigidBodyPlant within the same Diagram.
class IiwaStateFeedbackPlanSource : public systems::LeafSystem<double> {
// IMPORTANT NOTE : This component is not to be moved out of dev. See
// issue #5736 for more details.
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStateFeedbackPlanSource)

  explicit IiwaStateFeedbackPlanSource(const std::string& model_path,
                                       const double update_interval = 0.001);
  ~IiwaStateFeedbackPlanSource() override;

  const systems::InputPortDescriptor<double>& get_input_port_plan() const {
    return this->get_input_port(input_port_plan_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_state() const {
    return this->get_input_port(input_port_state_);
  }

  const systems::OutputPortDescriptor<double>&
  get_output_port_state_trajectory() const {
    return this->get_output_port(output_port_state_trajectory_);
  }

  const systems::OutputPortDescriptor<double>&
  get_output_port_acceleration_trajectory() const {
    return this->get_output_port(output_port_acceleration_trajectory_);
  }

 protected:
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port, int output_port) const final {
    return false;
  }

 private:
  struct InternalData;
  const int input_port_plan_{-1};
  const int input_port_state_{-1};
  const int output_port_state_trajectory_{-1};
  const int output_port_acceleration_trajectory_{-1};
  RigidBodyTree<double> tree_;
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
