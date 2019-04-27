
#include "drake/manipulation/robot_plan_runner/controller_systems.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using systems::BasicVector;
using systems::kVectorValued;

RobotController::RobotController(PlanType plan_type) : num_positions_(7) {
  switch (plan_type) {
    case PlanType::kJointSpaceController:
      this->set_name("JointSpaceController");
      plan_ = std::make_unique<JointSpacePlan>();
      break;
    case PlanType::kTaskSpaceController:
      this->set_name("TaskSpaceController");
      break;
  }

  // declare input and output ports
  input_port_idx_q_ =
      this->DeclareInputPort("q", kVectorValued, num_positions_).get_index();
  input_port_idx_v_ =
      this->DeclareInputPort("v", kVectorValued, num_positions_).get_index();
  input_port_idx_tau_ext_ =
      this->DeclareInputPort("tau_ext", kVectorValued, num_positions_)
          .get_index();
  input_port_idx_plan_data =
      this->DeclareAbstractInputPort("plan_data", Value<PlanData>())
          .get_index();
  this->DeclareVectorOutputPort("q_cmd", BasicVector<double>(num_positions_),
                                &RobotController::CalcCommands)
      .get_index();

  // resize data members
  q_cmd_.resize(num_positions_);
  tau_cmd_.resize(num_positions_);
};

void RobotController::CalcCommands(const systems::Context<double>& context,
                                   BasicVector<double>* q_tau_cmd) const {
  Eigen::VectorBlock<VectorX<double>> q_tau_vector =
      q_tau_cmd->get_mutable_value();

  const auto& q = this->get_input_port(input_port_idx_q_).Eval(context);
  const auto& v = this->get_input_port(input_port_idx_v_).Eval(context);
  const auto& tau_ext =
      this->get_input_port(input_port_idx_tau_ext_).Eval(context);
  const AbstractValue* plan_data_ptr =
      this->EvalAbstractInput(context, input_port_idx_plan_data);
  const auto& plan_data_value = plan_data_ptr->get_value<PlanData>();

  // check if the plan from input port is new.
  if(plan_data_value.plan_index > plan_index_current_) {
    plan_->UpdatePlan(plan_data_value);
    plan_index_current_ = plan_data_value.plan_index;
  }
  // TODO: record plan start time and take the difference.
  double t = context.get_time();

  plan_->Step(q, v, tau_ext, t, &q_cmd_, &tau_cmd_);
  q_tau_vector << q_cmd_, tau_cmd_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
