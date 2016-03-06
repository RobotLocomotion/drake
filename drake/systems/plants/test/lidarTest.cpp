
#include <cmath>
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  // unit test that sets up a lidar in a box room and verifies the returns
  auto rigid_body_sys = make_shared<RigidBodySystem>(
      getDrakePath() + "/systems/plants/test/lidarTest.sdf", DrakeJoint::FIXED);

  double t = 0;
  VectorXd x = VectorXd::Zero(0);
  VectorXd u = VectorXd::Zero(0);

  //  rigid_body_sys->getRigidBodyTree()->drawKinematicTree("/tmp/lidar.dot");

  auto distances = rigid_body_sys->output(t, x, u);

  // these parameters must match the sdf (i've also implicitly hard-coded the
  // box geometry from the sdf)
  const double min_yaw = -1.7;
  const double max_yaw = 1.7;
  const double tol = 0.05;  // todo: improve the tolerance (see #1712)

  for (int i = 0; i < distances.size(); i++) {
    double theta = min_yaw + (max_yaw - min_yaw) * i / (distances.size() - 1);
    if (abs(theta) >= M_PI / 2 + .05) {  // should not be hitting any wall.  //
                                         // todo: get rid of the .05 artifact
                                         // (see #1712)
      valuecheck(-1.0, distances(i));
    } else if (theta <= -M_PI / 4) {  // hitting the right wall
      valuecheck(-1.0 / sin(theta), distances(i), tol);
    } else if (theta >= M_PI / 4) {  // hitting the left wall
      valuecheck(1.0 / sin(theta), distances(i), tol);
    } else {  // hitting the front wall
      valuecheck(1 / cos(theta), distances(i), tol);
    }
  }

  // render for debugging:
  auto lcm = make_shared<lcm::LCM>();
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(
      lcm, rigid_body_sys->getRigidBodyTree());
  visualizer->output(t, x, u);

  return 0;
}
