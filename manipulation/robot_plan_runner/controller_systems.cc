
#include "drake/manipulation/robot_plan_runner/controller_systems.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using systems::BasicVector;
using systems::kVectorValued;

RobotController::RobotController(PlanTypes plan_type) : num_positions_(7) {
  switch(plan_type) {
    case PlanTypes::kJointSpaceController:
      this->set_name("JointSpaceController");
      plan_ = std::make_unique<JointSpacePlan>();
      break;
    case PlanTypes::kTaskSpaceController:
      this->set_name("TaskSpaceController");
      break;
  }

  input_port_idx_q_ =
      this->DeclareInputPort("q", kVectorValued, num_positions_).get_index();
  input_port_idx_v_ =
      this->DeclareInputPort("v", kVectorValued, num_positions_).get_index();
  input_port_idx_tau_ext_ =
      this->DeclareInputPort("tau_ext", kVectorValued, num_positions_)
          .get_index();
  output_port_idx_cmd_ = this->DeclareVectorOutputPort(
                                   "q_cmd", BasicVector<double>(num_positions_),
                                   &RobotController::CalcCommands)
                               .get_index();

  q_cmd_.resize(num_positions_);
  tau_cmd_.resize(num_positions_);
};


void RobotController::CalcCommands(
    const systems::Context<double>& context,
    BasicVector<double>* q_tau_cmd) const {
  Eigen::VectorBlock<VectorX<double>> q_tau_vector =
      q_tau_cmd->get_mutable_value();

  auto q = this->get_input_port(input_port_idx_q_).Eval(context);
  auto v = this->get_input_port(input_port_idx_v_).Eval(context);
  auto tau_ext = this->get_input_port(input_port_idx_tau_ext_).Eval(context);

  // TODO: record plan start time and take the difference.
  double t = context.get_time();

  plan_->Step(q, v, tau_ext, t, &q_cmd_, &tau_cmd_);
  q_tau_vector << q_cmd_, tau_cmd_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
