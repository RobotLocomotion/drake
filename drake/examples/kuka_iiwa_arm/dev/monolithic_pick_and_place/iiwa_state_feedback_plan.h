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
namespace pick_and_place {

/// This class implements a source of joint positions for an iiwa arm.
/// It has two input ports, one for robot_plan_t messages containing a
/// plan to follow, and another vector valued port for directly providing
/// state feedback.
///
/// If no plan has been received, the system will create an initial
/// plan on the first unrestricted state update which commands the arm
/// to hold at the measured position.
///
/// This class is largely similar to IiwaPlanSource in its functionality
/// but can be hooked up by direct state feedback and avoids a direct
/// feedthrough thus eliminating algebriac loops when used in conjunction
/// with a RigidBodyPlant within the same Diagram.

class IiwaStateFeedbackTrajectoryGenerator : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStateFeedbackTrajectoryGenerator)

  explicit IiwaStateFeedbackTrajectoryGenerator(const std::string& model_path,
                                                const double update_interval = 0.001);
  ~IiwaStateFeedbackTrajectoryGenerator() override;

  const systems::InputPortDescriptor<double>& get_input_port_plan() const {
    return this->get_input_port(input_port_plan_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_state() const {
    return this->get_input_port(input_port_state_);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_trajectory() const {
    return this->get_output_port(output_port_trajectory_);
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
  const int output_port_trajectory_{-1};
  RigidBodyTree<double> tree_;
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake