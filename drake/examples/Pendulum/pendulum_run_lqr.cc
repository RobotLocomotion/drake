#include <iostream>

#include "drake/common/drake_path.h"
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/system1/Simulation.h"
#include "drake/systems/controllers/LQR.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/feedback_system.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  auto p = std::make_shared<Pendulum>();

  Eigen::MatrixXd Q(2, 2);
  Q << 10, 0, 0, 1;
  Eigen::MatrixXd R(1, 1);
  R << 1;
  PendulumState<double> xG;
  xG.theta = M_PI;
  xG.thetadot = 0;
  PendulumInput<double> uG;
  uG.tau = 0;
  auto c = MakeTimeInvariantLqrSystem(*p, xG, uG, Q, R);
  auto v = std::make_shared<BotVisualizer<PendulumState> >(
      lcm, GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      drake::systems::plants::joints::kFixed);

  auto sys = cascade(feedback(p, c), v);

  SimulationOptions options;
  options.realtime_factor = 1.0;
  if (commandLineOptionExists(argv, argv + argc, "--non-realtime")) {
    options.warn_real_time_violation = true;
  }

  for (int i = 0; i < 5; i++) {
    Eigen::Vector2d x0 = toEigen(xG);
    x0 += Eigen::Vector2d::Random();
    simulate(*sys, 0, 5, x0, options);
  }
}
