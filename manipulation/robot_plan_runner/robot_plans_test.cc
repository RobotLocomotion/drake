
#include <vector>

#include "drake/manipulation/robot_plan_runner/robot_plans.h"


namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using trajectories::PiecewisePolynomial;
using std::endl;
using std::cout;


int do_main() {
  JointSpacePlan plan;

  Eigen::VectorXd t(3);
  t << 0, 1, 2;

  Eigen::MatrixXd q_knots(7, 3);
  q_knots.col(0) << 0, 0, 0, 0, 0, 0, 0;
  q_knots.col(1) << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
  q_knots.col(2) << 1, 1, 1, 1, 1, 1, 1;

  auto qtraj = PiecewisePolynomial<double>::Cubic(
      t, q_knots, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));

  std::unique_ptr<PiecewisePolynomial<double>> qtraj_ptr(&qtraj);
  plan.UpdatePlan(std::move(qtraj_ptr));

  Eigen::VectorXd q(7);
  Eigen::VectorXd v(7);
  Eigen::VectorXd tau_external(7);
  Eigen::VectorXd q_cmd(7);
  Eigen::VectorXd tau_cmd(7);

  std::vector<double> t_list = {0, 0.5, 1, 1.5, 2, 2.5};

  for(auto time : t_list) {
    plan.Step(q, v, tau_external, time, &q_cmd, &tau_cmd);
    cout << time << endl << q_cmd << endl << tau_cmd << endl;
  }

  return 0;
};

} // namespace drake
} // namespace manipulation
} // namespace drake


int main() {
 return drake::manipulation::robot_plan_runner::do_main();
};