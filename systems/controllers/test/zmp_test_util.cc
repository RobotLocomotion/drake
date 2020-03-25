#include "drake/systems/controllers/test/zmp_test_util.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace controllers {

using trajectories::PiecewisePolynomial;

ZMPTestTraj SimulateZMPPolicy(const ZMPPlanner& zmp_planner,
                              const Eigen::Vector4d& x0, double dt,
                              double extra_time_at_the_end) {
  const PiecewisePolynomial<double>& zmp_d = zmp_planner.get_desired_zmp();
  int N = static_cast<int>(
      (zmp_d.end_time() + extra_time_at_the_end - zmp_d.start_time())
      / dt);

  ZMPTestTraj traj(N);

  for (int i = 0; i < N; i++) {
    traj.time[i] = zmp_d.start_time() + i * dt;
    if (i == 0) {
      traj.x.col(i) = x0;
    } else {
      Eigen::Vector4d xd;
      xd << traj.x.col(i - 1).tail<2>(), traj.u.col(i - 1);
      traj.x.col(i) = traj.x.col(i - 1) + xd * dt;
    }

    traj.u.col(i) =
        zmp_planner.ComputeOptimalCoMdd(traj.time[i], traj.x.col(i));
    traj.cop.col(i) = zmp_planner.comdd_to_cop(traj.x.col(i), traj.u.col(i));

    traj.desired_zmp.col(i) = zmp_planner.get_desired_zmp(traj.time[i]);
    traj.nominal_com.col(i).head<2>() =
        zmp_planner.get_nominal_com(traj.time[i]);
    traj.nominal_com.col(i).segment<2>(2) =
        zmp_planner.get_nominal_comd(traj.time[i]);
    traj.nominal_com.col(i).tail<2>() =
        zmp_planner.get_nominal_comdd(traj.time[i]);
  }

  return traj;
}

std::vector<PiecewisePolynomial<double>> GenerateDesiredZMPTrajs(
    const std::vector<Eigen::Vector2d>& footsteps,
    double double_support_duration, double single_support_duration) {
  DRAKE_DEMAND(!footsteps.empty());

  std::vector<double> time_steps;
  std::vector<Eigen::MatrixXd> zmp_d;

  Eigen::Vector4d x0(0, 0, 0, 0);
  double time = 0;

  time_steps.push_back(time);
  zmp_d.push_back(footsteps.front());
  time += single_support_duration;
  time_steps.push_back(time);
  zmp_d.push_back(footsteps.front());

  for (int i = 1; i < static_cast<int>(footsteps.size()); ++i) {
    time += double_support_duration;
    time_steps.push_back(time);
    zmp_d.push_back(footsteps[i]);

    time += single_support_duration;
    time_steps.push_back(time);
    zmp_d.push_back(footsteps[i]);
  }

  std::vector<PiecewisePolynomial<double>> zmp_trajs;
  zmp_trajs.reserve(3);
  zmp_trajs.push_back(
      PiecewisePolynomial<double>::ZeroOrderHold(time_steps, zmp_d));
  zmp_trajs.push_back(
      PiecewisePolynomial<double>::FirstOrderHold(time_steps, zmp_d));
  zmp_trajs.push_back(
      PiecewisePolynomial<double>::CubicShapePreserving(time_steps, zmp_d));

  return zmp_trajs;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
