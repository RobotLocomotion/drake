
#include <unordered_map>

#include "drake/manipulation/robot_plan_runner/plan_switcher.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using systems::kVectorValued;
using robot_plans::PlanData;
using robot_plans::PlanType;

PlanSwitcher::PlanSwitcher() {
  this->set_name("PlanSwitcher");

  input_port_idx_plan_data_ =
      this->DeclareAbstractInputPort("plan_data", Value<PlanData>())
          .get_index();
  // TODO: PlanData does not need to pass through this system. This system
  //  should only be responsible for the switching.
  this->DeclareAbstractOutputPort("joint_space_plan",
                                  &PlanSwitcher::CalcJointSpacePlan);
}

void PlanSwitcher::CalcJointSpacePlan(const systems::Context<double>& context,
                                      PlanData* output_plan_data) const {
  const AbstractValue* input =
      this->EvalAbstractInput(context, input_port_idx_plan_data_);
  const auto& input_plan_data = input->get_value<PlanData>();

  if (input_plan_data.plan_type == PlanType::kJointSpacePlan) {
    *output_plan_data = input_plan_data;
  }
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
