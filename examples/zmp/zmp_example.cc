#include <chrono>
#include <thread>

#include "drake/common/proto/call_python.h"
#include "drake/systems/controllers/test/zmp_test_util.h"

namespace drake {
namespace examples {
namespace zmp {
namespace {

void PlotResults(const systems::controllers::ZMPTestTraj& traj) {
  using common::CallPython;
  using common::ToPythonTuple;

  CallPython("figure", 1);
  CallPython("clf");
  CallPython("subplot", 2, 1, 1);
  CallPython("plot", traj.time.transpose(),
             traj.desired_zmp.row(0).transpose(), "r");
  CallPython("plot", traj.time.transpose(),
             traj.nominal_com.row(0).transpose(), "b");
  CallPython("plot", traj.time.transpose(), traj.cop.row(0).transpose(), "g");
  CallPython("plot", traj.time.transpose(), traj.x.row(0).transpose(), "c");
  CallPython("xlabel", "time [s]");
  CallPython("ylabel", "x [m]");
  CallPython("legend", ToPythonTuple("desired zmp", "planned com",
                                     "planned cop", "actual com"));

  CallPython("subplot", 2, 1, 2);
  CallPython("plot", traj.time.transpose(),
             traj.desired_zmp.row(1).transpose(), "r");
  CallPython("plot", traj.time.transpose(),
             traj.nominal_com.row(1).transpose(), "b");
  CallPython("plot", traj.time.transpose(), traj.cop.row(1).transpose(), "g");
  CallPython("plot", traj.time.transpose(), traj.x.row(1).transpose(), "c");
  CallPython("xlabel", "time [s]");
  CallPython("ylabel", "y [m]");
  CallPython("legend", ToPythonTuple("desired zmp", "planned com",
                                     "planned cop", "actual com"));
  // Give time for Python to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  CallPython("figure", 2);
  CallPython("clf");
  CallPython("subplot", 2, 1, 1);
  CallPython("plot", traj.time.transpose(),
             traj.nominal_com.row(2).transpose(), "b");
  CallPython("plot", traj.time.transpose(), traj.x.row(2).transpose(), "c");
  CallPython("xlabel", "time [s]");
  CallPython("ylabel", "xd [m/s]");
  CallPython("legend", ToPythonTuple("planned comd", "actual comd"));

  CallPython("subplot", 2, 1, 2);
  CallPython("plot", traj.time.transpose(),
             traj.nominal_com.row(3).transpose(), "b");
  CallPython("plot", traj.time.transpose(), traj.x.row(3).transpose(), "c");
  CallPython("xlabel", "time [s]");
  CallPython("ylabel", "yd [m/s]");
  CallPython("legend", ToPythonTuple("planned comd", "actual comd"));
  // Give time for Python to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  CallPython("figure", 3);
  CallPython("clf");
  CallPython("subplot", 2, 1, 1);
  CallPython("plot", traj.time.transpose(), traj.u.row(0).transpose(), "r");
  CallPython("plot", traj.time.transpose(),
             traj.nominal_com.row(4).transpose(), "b.");
  CallPython("xlabel", "time [s]");
  CallPython("ylabel", "xdd [m/s2]");
  CallPython("legend", ToPythonTuple("comdd from policy", "nominal comdd"));
  CallPython("subplot", 2, 1, 2);
  CallPython("plot", traj.time.transpose(), traj.u.row(1).transpose(), "r");
  CallPython("plot", traj.time.transpose(),
             traj.nominal_com.row(5).transpose(), "b.");
  CallPython("xlabel", "time [s]");
  CallPython("ylabel", "ydd [m/s2]");
  CallPython("legend", ToPythonTuple("comdd from policy", "nominal comdd"));
  // Give time for Python to plot.
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void do_main() {
  std::vector<Eigen::Vector2d> footsteps = {
      Eigen::Vector2d(0, 0),    Eigen::Vector2d(0.5, 0.1),
      Eigen::Vector2d(1, -0.1), Eigen::Vector2d(1.5, 0.1),
      Eigen::Vector2d(2, -0.1), Eigen::Vector2d(2.5, 0)};

  std::vector<trajectories::PiecewisePolynomial<double>> zmp_trajs =
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


