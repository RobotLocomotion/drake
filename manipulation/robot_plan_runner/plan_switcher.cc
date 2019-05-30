
#include <unordered_map>

#include "drake/manipulation/robot_plan_runner/plan_switcher.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using robot_plans::PlanData;
using robot_plans::PlanType;
using systems::kVectorValued;

PlanSwitcher::PlanSwitcher() {
  this->set_name("PlanSwitcher");

  input_port_idx_plan_data_ =
      this->DeclareAbstractInputPort("plan_data", Value<PlanData>())
          .get_index();
  this->DeclareAbstractOutputPort("port_switch_index",
                                  &PlanSwitcher::CalcSwitchIndex);
}

void PlanSwitcher::CalcSwitchIndex(
    const systems::Context<double>& context,
    systems::InputPortIndex* plan_switch_index) const {
  const AbstractValue* input =
      this->EvalAbstractInput(context, input_port_idx_plan_data_);
  *plan_switch_index = systems::InputPortIndex(
      static_cast<int>(input->template get_value<PlanData>().plan_type));
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
