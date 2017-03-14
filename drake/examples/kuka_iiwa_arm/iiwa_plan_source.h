#pragma once

#include <memory>
#include <string>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// This class implements a source of joint positions for an iiwa arm.
/// It has two input ports, one for robot_plan_t messages containing a
/// plan to follow, and another port for lcmt_iiwa_status messages.
///
/// If no plan has been received, the system will create an initial
/// plan on the first unrestricted state update which commands the arm
/// to hold at the measured position.
///
/// It is an error to attempt to read from the output port if a valid
/// status message is not available.

class IiwaPlanSource : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaPlanSource)

  explicit IiwaPlanSource(const std::string& model_path);
  ~IiwaPlanSource() override;

  const systems::InputPortDescriptor<double>& get_plan_input_port() const {
    return this->get_input_port(plan_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_status_input_port() const {
    return this->get_input_port(status_input_port_);
  }

 protected:
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

 private:
  struct PlanData;

  const int plan_input_port_{};
  const int status_input_port_{};
  RigidBodyTree<double> tree_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
