#include <chrono>
#include <thread>

#include "drake/common/proto/call_matlab.h"
#include "drake/systems/controllers/test/zmp_test_util.h"

namespace drake {
namespace examples {
namespace zmp {
namespace {

void PlotResults(const systems::controllers::ZMPTestTraj& traj) {
  using drake::common::CallMatlab;

  CallMatlab("figure", 1);
  CallMatlab("clf");
  CallMatlab("subplot", 2, 1, 1);
  CallMatlab("hold", "on");
  CallMatlab("plot", traj.time, traj.desired_zmp.row(0), "r");
  CallMatlab("plot", traj.time, traj.nominal_com.row(0), "b");
  CallMatlab("plot", traj.time, traj.cop.row(0), "g");
  CallMatlab("plot", traj.time, traj.x.row(0), "c");
  CallMatlab("xlabel", "time [s]");
  CallMatlab("ylabel", "x [m]");
  CallMatlab("legend", "desired zmp", "planned com", "planned cop",
                "actual com");

  CallMatlab("subplot", 2, 1, 2);
  CallMatlab("hold", "on");
  CallMatlab("plot", traj.time, traj.desired_zmp.row(1), "r");
  CallMatlab("plot", traj.time, traj.nominal_com.row(1), "b");
  CallMatlab("plot", traj.time, traj.cop.row(1), "g");
  CallMatlab("plot", traj.time, traj.x.row(1), "c");
  CallMatlab("xlabel", "time [s]");
  CallMatlab("ylabel", "y [m]");
  CallMatlab("legend", "desired zmp", "planned com", "planned cop",
                "actual com");
  // Give time for matlab to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  CallMatlab("figure", 2);
  CallMatlab("clf");
  CallMatlab("subplot", 2, 1, 1);
  CallMatlab("hold", "on");
  CallMatlab("plot", traj.time, traj.nominal_com.row(2), "b");
  CallMatlab("plot", traj.time, traj.x.row(2), "c");
  CallMatlab("xlabel", "time [s]");
  CallMatlab("ylabel", "xd [m/s]");
  CallMatlab("legend", "planned comd", "actual comd");

  CallMatlab("subplot", 2, 1, 2);
  CallMatlab("hold", "on");
  CallMatlab("plot", traj.time, traj.nominal_com.row(3), "b");
  CallMatlab("plot", traj.time, traj.x.row(3), "c");
  CallMatlab("xlabel", "time [s]");
  CallMatlab("ylabel", "yd [m/s]");
  CallMatlab("legend", "planned comd", "actual comd");
  // Give time for matlab to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  CallMatlab("figure", 3);
  CallMatlab("clf");
  CallMatlab("subplot", 2, 1, 1);
  CallMatlab("hold", "on");
  CallMatlab("plot", traj.time, traj.u.row(0), "r");
  CallMatlab("plot", traj.time, traj.nominal_com.row(4), "b.");
  CallMatlab("xlabel", "time [s]");
  CallMatlab("ylabel", "xdd [m/s2]");
  CallMatlab("legend", "comdd from policy", "nominal comdd");

  CallMatlab("subplot", 2, 1, 2);
  CallMatlab("hold", "on");
  CallMatlab("plot", traj.time, traj.u.row(1), "r");
  CallMatlab("plot", traj.time, traj.nominal_com.row(5), "b.");
  CallMatlab("xlabel", "time [s]");
  CallMatlab("ylabel", "ydd [m/s2]");
  CallMatlab("legend", "comdd from policy", "nominal comdd");
  // Give time for matlab to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void do_main() {
  std::vector<Eigen::Vector2d> footsteps = {
      Eigen::Vector2d(0, 0),    Eigen::Vector2d(0.5, 0.1),
      Eigen::Vector2d(1, -0.1), Eigen::Vector2d(1.5, 0.1),
      Eigen::Vector2d(2, -0.1), Eigen::Vector2d(2.5, 0)};

  std::vector<PiecewisePolynomial<double>> zmp_trajs =
      systems::controllers::GenerateDesiredZMPTrajs(footsteps, 0.5, 1);

  Eigen::Vector4d x0(0, 0, 0, 0);
  double z = 1;

  systems::controllers::ZMPPlanner zmp_planner;
  zmp_planner.Plan(zmp_trajs[0], x0, z);

  double sample_dt = 0.01;

  // Perturb the initial state a bit.
  x0 << 0, 0, 0.2, -0.1;
  systems::controllers::ZMPTestTraj result =
      systems::controllers::SimulateZMPPolicy(zmp_planner, x0, sample_dt, 2);

  PlotResults(result);
}

}  // namespace
}  // namespace zmp
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::zmp::do_main();
  return 0;
}


