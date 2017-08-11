#include "drake/examples/pr2/src/utils/plan_status_lcm.h"

#include "drake/common/drake_assert.h"
#include "robotlocomotion/plan_status_t.hpp"

namespace drake {
namespace examples {
namespace pr2{

using systems::BasicVector;
using systems::Context;

PlanStatusSender::PlanStatusSender() {
  this->DeclareInputPort(systems::kVectorValued, 6);
  this->DeclareAbstractOutputPort(&PlanStatusSender::MakeOutputStatus,
                                  &PlanStatusSender::OutputStatus);
}

robotlocomotion::plan_status_t PlanStatusSender::MakeOutputStatus() const {
  robotlocomotion::plan_status_t msg{};
  return msg;
}

void PlanStatusSender::OutputStatus(const Context<double>& context,
                                    robotlocomotion::plan_status_t* output) const {
  robotlocomotion::plan_status_t& status = *output;

  status.utime = context.get_time() * 1e6;
  const BasicVector<double>* state = this->EvalVectorInput(context, 0);
  status.execution_status = state->GetAtIndex(0);
  status.last_plan_msg_utime = state->GetAtIndex(1) * 1e6;
  status.last_plan_start_utime = state->GetAtIndex(2) * 1e6;
  status.plan_type = state->GetAtIndex(3);
  status.recovery_enabled = state->GetAtIndex(4) != 0.0;
  status.bracing_enabled = state->GetAtIndex(5) != 0.0;
  }

}  // namespace pr2
}  // namespace examples
}  // namespace drake
