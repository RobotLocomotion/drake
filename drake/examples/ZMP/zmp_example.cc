#include <chrono>
#include <thread>

#include "drake/examples/ZMP/zmp_test_util.h"
#include "drake/lcm/lcm_call_matlab.h"

void PlotResults(const drake::examples::zmp::ZMPTestTraj& traj) {
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
  // Give time for matlab to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));

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
  // Give time for matlab to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));

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
  // Give time for matlab to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main() {
  std::vector<Eigen::Vector2d> footsteps = {
      Eigen::Vector2d(0, 0),    Eigen::Vector2d(0.5, 0.1),
      Eigen::Vector2d(1, -0.1), Eigen::Vector2d(1.5, 0.1),
      Eigen::Vector2d(2, -0.1), Eigen::Vector2d(2.5, 0)};

  std::vector<PiecewisePolynomial<double>> zmp_trajs =
      drake::examples::zmp::GenerateDesiredZMPTrajs(footsteps, 0.5, 1);

  Eigen::Vector4d x0(0, 0, 0, 0);
  double z = 1;

  drake::systems::ZMPPlanner zmp_planner;
  zmp_planner.Plan(zmp_trajs[0], x0, z);

  double sample_dt = 0.01;

  // Perturb the initial state a bit.
  x0 << 0, 0, 0.2, -0.1;
  drake::examples::zmp::ZMPTestTraj result =
      drake::examples::zmp::SimulateZMPPolicy(zmp_planner, x0, sample_dt, 2);

  PlotResults(result);

  return 0;
}
