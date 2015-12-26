#include <iostream>
#include "Quadrotor.h"
#include "Simulation.h"
#include "LQR.h"
#include "BotVisualizer.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);

  if(!lcm->good())
    return 1;

  auto quad = std::make_shared<Quadrotor>();

  const int num_positions = quad->num_states/2;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(quad->num_states, quad->num_states);
  Q.topLeftCorner(num_positions, num_positions) = 10.0 * Eigen::MatrixXd::Identity(num_positions, num_positions);
  Eigen::MatrixXd R = 0.1*Eigen::MatrixXd::Identity(quad->num_inputs, quad->num_inputs);
  QuadrotorState<double> xG;  xG.z = 1;
  QuadrotorInput<double> uG;  uG.w1 = quad->m * quad->g * 0.25; uG.w2 = uG.w1; uG.w3 = uG.w1; uG.w4 = uG.w1;
  auto c = timeInvariantLQR(*quad,xG,uG,Q,R);
  auto v = std::make_shared<Drake::BotVisualizer<QuadrotorState> >(lcm,Drake::getDrakePath()+"/examples/Quadrotor/quadrotor.urdf",DrakeJoint::ROLLPITCHYAW);

  auto sys = cascade(feedback(quad,c),v);

  SimulationOptions options;
  options.realtime_factor=1.0;
  options.initial_step_size = 0.005;

  for (int i=0; i<5; i++) {
    Eigen::Matrix<double,12,1> x0 = xG;
    x0 += static_cast<Eigen::Matrix<double,12,1> >(quad->getRandomState<double>());
    simulate(*sys,0,10,x0,options);
  }

  // todo: change this back to runLCM instead of just simulate
}

