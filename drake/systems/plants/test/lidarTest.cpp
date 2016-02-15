
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/BotVisualizer.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  // unit test that sets up a lidar in a box room and verifies the returns
  auto rigid_body_sys = make_shared<RigidBodySystem>(getDrakePath() + "/systems/plants/test/lidarTest.sdf",DrakeJoint::FIXED);
  auto lcm = make_shared<lcm::LCM>();
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,rigid_body_sys->getRigidBodyTree());

  visualizer->output(0,VectorXd::Zero(0),VectorXd::Zero(0));

  return 0;
}
