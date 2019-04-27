#include "drake/manipulation/robot_plan_runner/robot_plans.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {



void JointSpacePlan::UpdatePlan(const PlanData& plan_data) {
  DRAKE_ASSERT(plan_data.plan_type == PlanType::kJointSpaceController);
  q_traj_.reset(&plan_data.joint_traj.value());
};

void JointSpacePlan::Step(const Eigen::Ref<const Eigen::VectorXd>& q,
          const Eigen::Ref<const Eigen::VectorXd>& v,
          const Eigen::Ref<const Eigen::VectorXd>& tau_external, double t,
          Eigen::VectorXd* const q_cmd,
          Eigen::VectorXd* const tau_cmd) const {
  if (q_traj_) {
    // if q_traj_ is not NULL.
    *q_cmd = q_traj_->value(t);
    *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
  }
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
