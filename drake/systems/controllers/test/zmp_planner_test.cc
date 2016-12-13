#include <iostream>

#include "drake/systems/controllers/zmp_planner.h"

#include "drake/lcm/lcm_call_matlab.h"

struct ZMPTestTraj {
  Eigen::Matrix<double, 1, Eigen::Dynamic> time;
  Eigen::Matrix<double, 6, Eigen::Dynamic> nominal_com;
  Eigen::Matrix<double, 2, Eigen::Dynamic> desired_zmp;

  Eigen::Matrix<double, 4, Eigen::Dynamic> x;
  Eigen::Matrix<double, 2, Eigen::Dynamic> u;
  Eigen::Matrix<double, 2, Eigen::Dynamic> cop;

  ZMPTestTraj(int N) {
    time.resize(N);
    nominal_com.resize(6, N);
    desired_zmp.resize(2, N);
    x.resize(4, N);
    u.resize(2, N);
    cop.resize(2, N);
  }
};

void PlotResults(const ZMPTestTraj& traj) {
  using drake::lcm::LcmCallMatlab;

  LcmCallMatlab("figure", 1);
  LcmCallMatlab("clf");
  LcmCallMatlab("subplot", 2, 1, 1);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", traj.time, traj.desired_zmp.row(0), "r");
  LcmCallMatlab("plot", traj.time, traj.nominal_com.row(0), "b");
  LcmCallMatlab("plot", traj.time, traj.cop.row(0), "g");
  LcmCallMatlab("plot", traj.time, traj.x.row(0), "c");
  LcmCallMatlab("xlabel", "time [s]");
  LcmCallMatlab("ylabel", "x [m]");
  LcmCallMatlab("legend", "desired zmp", "planned com", "planned cop",
                "actual com");

  LcmCallMatlab("subplot", 2, 1, 2);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", traj.time, traj.desired_zmp.row(1), "r");
  LcmCallMatlab("plot", traj.time, traj.nominal_com.row(1), "b");
  LcmCallMatlab("plot", traj.time, traj.cop.row(1), "g");
  LcmCallMatlab("plot", traj.time, traj.x.row(1), "c");
  LcmCallMatlab("xlabel", "time [s]");
  LcmCallMatlab("ylabel", "y [m]");
  LcmCallMatlab("legend", "desired zmp", "planned com", "planned cop",
                "actual com");

  LcmCallMatlab("figure", 2);
  LcmCallMatlab("clf");
  LcmCallMatlab("subplot", 2, 1, 1);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", traj.time, traj.nominal_com.row(2), "b");
  LcmCallMatlab("plot", traj.time, traj.x.row(2), "c");
  LcmCallMatlab("xlabel", "time [s]");
  LcmCallMatlab("ylabel", "xd [m/s]");
  LcmCallMatlab("legend", "planned comd", "actual comd");

  LcmCallMatlab("subplot", 2, 1, 2);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", traj.time, traj.nominal_com.row(3), "b");
  LcmCallMatlab("plot", traj.time, traj.x.row(3), "c");
  LcmCallMatlab("xlabel", "time [s]");
  LcmCallMatlab("ylabel", "yd [m/s]");
  LcmCallMatlab("legend", "planned comd", "actual comd");

  LcmCallMatlab("figure", 3);
  LcmCallMatlab("clf");
  LcmCallMatlab("subplot", 2, 1, 1);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", traj.time, traj.u.row(0), "r");
  LcmCallMatlab("plot", traj.time, traj.nominal_com.row(4), "b.");
  LcmCallMatlab("xlabel", "time [s]");
  LcmCallMatlab("ylabel", "xdd [m/s2]");
  LcmCallMatlab("legend", "comdd from policy", "nominal comdd");

  LcmCallMatlab("subplot", 2, 1, 2);
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", traj.time, traj.u.row(1), "r");
  LcmCallMatlab("plot", traj.time, traj.nominal_com.row(5), "b.");
  LcmCallMatlab("xlabel", "time [s]");
  LcmCallMatlab("ylabel", "ydd [m/s2]");
  LcmCallMatlab("legend", "comdd from policy", "nominal comdd");
}

void PlotNominal(const drake::systems::ZMPPlanner& zmp_planner, double dt) {
  const PiecewisePolynomial<double>& zmp_d = zmp_planner.get_desired_zmp();
  int N =
      static_cast<int>((zmp_d.getEndTime() + 1 - zmp_d.getStartTime()) / dt);

  ZMPTestTraj traj(N);

  for (int i = 0; i < N; i++) {
    traj.time[i] = zmp_d.getStartTime() + i * dt;
    traj.x.col(i).head<2>() = traj.nominal_com.col(i).head<2>() =
        zmp_planner.get_nominal_com(traj.time[i]);
    traj.x.col(i).tail<2>() = traj.nominal_com.col(i).segment<2>(2) =
        zmp_planner.get_nominal_comd(traj.time[i]);
    traj.nominal_com.col(i).tail<2>() =
        zmp_planner.get_nominal_comdd(traj.time[i]);

    traj.desired_zmp.col(i) = zmp_planner.get_desired_zmp(traj.time[i]);
    traj.u.col(i) =
        zmp_planner.ComputeOptimalCoMdd(traj.time[i], traj.x.col(i));
    traj.cop.col(i) = zmp_planner.comdd_to_cop(traj.x.col(i), traj.u.col(i));
  }

  PlotResults(traj);
}

void TestSim(const drake::systems::ZMPPlanner& zmp_planner,
             const Eigen::Vector4d& x0, double dt) {
  const PiecewisePolynomial<double>& zmp_d = zmp_planner.get_desired_zmp();
  int N =
      static_cast<int>((zmp_d.getEndTime() + 1 - zmp_d.getStartTime()) / dt);

  ZMPTestTraj traj(N);

  for (int i = 0; i < N; i++) {
    traj.time[i] = zmp_d.getStartTime() + i * dt;
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

  PlotResults(traj);
}

PiecewisePolynomial<double> GenerateDesiredZMPTraj() {
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
  time += ss;
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

  // PiecewisePolynomial<double> zmp_traj =
  //    PiecewisePolynomial<double>::ZeroOrderHold(Ts, zmp_d);
  // PiecewisePolynomial<double> zmp_traj =
  //  PiecewisePolynomial<double>::FirstOrderHold(Ts, zmp_d);
  PiecewisePolynomial<double> zmp_traj =
      PiecewisePolynomial<double>::Pchip(Ts, zmp_d);

  return zmp_traj;
}

int main() {
  PiecewisePolynomial<double> zmp_traj = GenerateDesiredZMPTraj();

  double z = 1;
  double sample_dt = 0.01;
  Eigen::Vector4d x0(0, 0, 0, 0);

  drake::systems::ZMPPlanner zmp_planner;
  zmp_planner.Plan(zmp_traj, x0, z);

  // PlotNominal(zmp_planner, sample_dt);

  TestSim(zmp_planner, Eigen::Vector4d(0.1, -0.05, 0.1, 0.1), sample_dt);

  return 0;
}
