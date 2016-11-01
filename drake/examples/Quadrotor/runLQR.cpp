#include <iostream>

#include "drake/common/drake_path.h"
#include "drake/examples/Quadrotor/Quadrotor.h"
#include "drake/system1/Simulation.h"
#include "drake/systems/controllers/LQR.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/feedback_system.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace Eigen;
using namespace drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);

  if (!lcm->good()) return 1;

  auto quad = std::make_shared<Quadrotor>();

  const int num_states = getNumStates(*quad);
  const int num_positions = num_states / 2;
  const int num_inputs = getNumInputs(*quad);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(num_states, num_states);
  Q.topLeftCorner(num_positions, num_positions) =
      10.0 * Eigen::MatrixXd::Identity(num_positions, num_positions);
  Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(num_inputs, num_inputs);
  QuadrotorState<double> xG;
  xG.z = 1;
  QuadrotorInput<double> uG;
  uG.w1 = quad->m * quad->g * 0.25;
  uG.w2 = uG.w1;
  uG.w3 = uG.w1;
  uG.w4 = uG.w1;
  auto c = MakeTimeInvariantLqrSystem(*quad, xG, uG, Q, R);
  auto v = std::make_shared<BotVisualizer<QuadrotorState> >(
      lcm, GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      drake::systems::plants::joints::kRollPitchYaw);

  auto sys = cascade(feedback(quad, c), v);

  SimulationOptions options;
  options.realtime_factor = 1.0;
  options.initial_step_size = 0.005;
  if (commandLineOptionExists(argv, argv + argc, "--non-realtime")) {
    options.warn_real_time_violation = true;
  }

  for (int i = 0; i < 5; i++) {
    Eigen::Matrix<double, 12, 1> x0 = toEigen(xG);
    x0 += toEigen(getRandomVector<QuadrotorState>());
    simulate(*sys, 0, 10, x0, options);
  }

  // todo: change this back to runLCM instead of just simulate
}
