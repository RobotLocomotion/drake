#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

/*
 * Tracks joint angle references.
 */
class JointSpacePlan : public PlanBase {
 public:
  JointSpacePlan() : PlanBase(PlanType::kJointSpacePlan, 7) {};

  void Step(const Eigen::Ref<const Eigen::VectorXd> &q,
            const Eigen::Ref<const Eigen::VectorXd> &v,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external,
            double control_period,
            double t,
            const PlanData &plan_data,
            EigenPtr<Eigen::VectorXd> q_cmd,
            EigenPtr<Eigen::VectorXd> tau_cmd) const override;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake