#include "drake/manipulation/robot_plan_runner/robot_plans.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

void JointSpacePlan::Step(const Eigen::Ref<const Eigen::VectorXd>& q,
          const Eigen::Ref<const Eigen::VectorXd>& v,
          const Eigen::Ref<const Eigen::VectorXd>& tau_external, double t,
          const PlanData& plan_data,
          Eigen::VectorXd* const q_cmd,
          Eigen::VectorXd* const tau_cmd) const {
  DRAKE_ASSERT(plan_data.plan_type == this->get_plan_type());
  const auto& q_traj = plan_data.joint_traj.value();
  *q_cmd = q_traj.value(t);
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
};


}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
