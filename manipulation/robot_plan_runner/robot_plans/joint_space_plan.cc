#include "drake/manipulation/robot_plan_runner/robot_plans/joint_space_plan.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using Eigen::VectorXd;

void JointSpacePlan::Step(const Eigen::Ref<const Eigen::VectorXd> &,
                          const Eigen::Ref<const Eigen::VectorXd> &,
                          const Eigen::Ref<const Eigen::VectorXd> &,
                          double,
                          double t,
                          const PlanData &plan_data,
                          EigenPtr<VectorXd> q_cmd,
                          EigenPtr<VectorXd> tau_cmd) const {
  DRAKE_THROW_UNLESS(plan_data.plan_type == this->get_plan_type());
  const auto &q_traj = plan_data.joint_traj.value();
  *q_cmd = q_traj.value(t);
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake