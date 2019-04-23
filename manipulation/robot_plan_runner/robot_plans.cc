#include <vector>

#include <drake/common/trajectories/piecewise_polynomial.h>
#include "drake/manipulation/robot_plan_runner/robot_plans.h"


namespace drake {
namespace manipulation {
namespace robot_plan_runner {

class PlanBase {
 public:
  PlanBase() = default;
  virtual ~PlanBase() = 0;

  virtual void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &tau_external,
                    double t, Eigen::VectorXd *const q_commanded,
                    Eigen::VectorXd *const tau_commanded) = 0;

  virtual void UpdatePlan(
      drake::trajectories::PiecewisePolynomial<double> q_traj) = 0;


};

//
//class JointSpacePlan : public PlanBase {
// public:
//  JointSpacePlan
//}

} // namespace robot_plan_runner
} // namespace manipulation
} // namespace drake

