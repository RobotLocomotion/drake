#include <iostream>

#include "drake/systems/controllers/zmp_planner.h"

#include "drake/lcm/lcm_call_matlab.h"

void TestZMPPlanner(const PiecewisePolynomial<double>& zmp_d,
    const Eigen::Vector4d& x0, double height) {
  drake::systems::ZMPPlanner zmp_planner;
  zmp_planner.Plan(zmp_d, x0, height);

  double dt = 0.01;
  int N = static_cast<int>(
      (zmp_d.getEndTime() + 1 - zmp_d.getStartTime()) / dt);

  Eigen::VectorXd time(N);
  Eigen::MatrixXd zmp(2, N);
  Eigen::MatrixXd com(2, N);
  Eigen::MatrixXd comd(2, N);
  Eigen::MatrixXd comdd(2, N);
  Eigen::MatrixXd u(2, N);
  Eigen::Vector4d x;

  for (int i = 0; i < N; i++) {
    time[i] = zmp_d.getStartTime() + i * dt;
    zmp.col(i) = zmp_planner.get_desired_zmp(time[i]);
    com.col(i) = zmp_planner.get_nominal_com(time[i]);
    comd.col(i) = zmp_planner.get_nominal_comd(time[i]);
    comdd.col(i) = zmp_planner.get_nominal_comdd(time[i]);

    x << com.col(i), comd.col(i);
    u.col(i) = zmp_planner.ComputeOptimalCoMdd(time[i], x);
  }

  using drake::lcm::LcmCallMatlab;
  LcmCallMatlab("figure", 1);
  LcmCallMatlab("clf");
  LcmCallMatlab("subplot", 2, 1, 1);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", time, zmp.row(0), "r");
  LcmCallMatlab("plot", time, com.row(0), "b");
  LcmCallMatlab("subplot", 2, 1, 2);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", time, zmp.row(1), "r");
  LcmCallMatlab("plot", time, com.row(1), "b");

  LcmCallMatlab("figure", 2);
  LcmCallMatlab("clf");
  LcmCallMatlab("subplot", 2, 1, 1);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", time, u.row(0), "r");
  LcmCallMatlab("plot", time, comdd.row(0), "b.");
  LcmCallMatlab("subplot", 2, 1, 2);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", time, u.row(1), "r");
  LcmCallMatlab("plot", time, comdd.row(1), "b.");
}

int main() {
  std::vector<double> Ts;
  std::vector<Eigen::MatrixXd> zmp_d;

  Eigen::Vector4d x0(0, 0, 0, 0);
  double x = 0;
  double y = 0.5;
  double time = 0;
  double ss = 1;
  double ds = 0.5;
  Ts.push_back(time);
  zmp_d.push_back(x0.head<2>());

  for (int i = 0; i < 5; i++) {
    time += ds;
    x += 0.5;
    y *= -1;
    Ts.push_back(time);
    zmp_d.push_back(Eigen::Vector2d(x, y));

    time += ss;
    Ts.push_back(time);
    zmp_d.push_back(Eigen::Vector2d(x, y));
  }

  PiecewisePolynomial<double> zmp_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(Ts, zmp_d);
  // PiecewisePolynomial<double> zmp_traj =
  //  PiecewisePolynomial<double>::FirstOrderHold(Ts, zmp_d);
  // PiecewisePolynomial<double> zmp_traj =
  //  PiecewisePolynomial<double>::Pchip(Ts, zmp_d);

  double z = 1;
  TestZMPPlanner(zmp_traj, x0, z);

  return 0;
}
