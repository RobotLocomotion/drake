
#include <unordered_map>

#include "drake/manipulation/robot_plan_runner/controller_systems.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using systems::BasicVector;
using systems::kVectorValued;

RobotController::RobotController(PlanType plan_type) : num_positions_(7) {
  switch (plan_type) {
    case PlanType::kJointSpacePlan:
      this->set_name("JointSpaceController");
      plan_ = std::make_unique<JointSpacePlan>();
      break;
    case PlanType::kTaskSpacePlan:
      this->set_name("TaskSpaceController");
      break;
    case PlanType::kEmptyPlan:
      break;
  }

  // input ports
  input_port_idx_q_ =
      this->DeclareInputPort("q", kVectorValued, num_positions_).get_index();
  input_port_idx_v_ =
      this->DeclareInputPort("v", kVectorValued, num_positions_).get_index();
  input_port_idx_tau_ext_ =
      this->DeclareInputPort("tau_external", kVectorValued, num_positions_)
          .get_index();
  input_port_idx_plan_data_ =
      this->DeclareAbstractInputPort("plan_data", Value<PlanData>{})
          .get_index();

  // output port
  this->DeclareVectorOutputPort("q_tau_cmd",
                                BasicVector<double>(2 * num_positions_),
                                &RobotController::CalcCommands);
  // resize data members
  q_cmd_.resize(num_positions_);
  tau_cmd_.resize(num_positions_);
};

void RobotController::CalcCommands(const systems::Context<double>& context,
                                   BasicVector<double>* q_tau_cmd) const {
  const AbstractValue* plan_data_ptr =
      this->EvalAbstractInput(context, input_port_idx_plan_data_);
  const auto& plan_data = plan_data_ptr->get_value<PlanData>();

  DRAKE_THROW_UNLESS(plan_data.plan_type == plan_->get_plan_type());

  // evaluate robot state input ports
  Eigen::VectorBlock<VectorX<double>> q_tau_vector =
      q_tau_cmd->get_mutable_value();
  const auto& q = this->get_input_port(input_port_idx_q_).Eval(context);
  const auto& v = this->get_input_port(input_port_idx_v_).Eval(context);
  const auto& tau_ext =
      this->get_input_port(input_port_idx_tau_ext_).Eval(context);

  // check if the plan from input port is new.
  if (plan_data.plan_signature > -1 &&
      plan_data.plan_signature != plan_signature_current_) {
    plan_signature_current_ = plan_data.plan_signature;
    t_start_current_ = context.get_time();
  }

  double t = context.get_time() - t_start_current_;

  plan_->Step(q, v, tau_ext, t, plan_data, &q_cmd_, &tau_cmd_);
  q_tau_vector << q_cmd_, tau_cmd_;

};

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

void PlanSwitcher::CalcJointSpacePlan(
    const systems::Context<double>& context,
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
