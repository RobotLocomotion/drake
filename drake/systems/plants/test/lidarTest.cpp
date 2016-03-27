
#include <cmath>
#include "lcmtypes/bot_core/planar_lidar_t.hpp"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  // unit test that sets up a lidar in a box room and verifies the returns

  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(getDrakePath() + "/systems/plants/test/lidarTest.sdf", DrakeJoint::FIXED);

  double t = 0;
  VectorXd x = VectorXd::Zero(rigid_body_sys->getNumStates());
  VectorXd u = VectorXd::Zero(rigid_body_sys->getNumInputs());

  //  rigid_body_sys->getRigidBodyTree()->drawKinematicTree("/tmp/lidar.dot");

  auto distances = rigid_body_sys->output(t, x, u);

  // these parameters must match the sdf (i've also implicitly hard-coded the
  // box geometry from the sdf)
  const double min_yaw = -1.7;
  const double max_yaw = 1.7;
  const double max_range = 25.0;
  const double tol = 0.01;

  for (int i = 0; i < distances.size(); i++) {
    double theta = min_yaw + (max_yaw - min_yaw) * i / (distances.size() - 1);

    if (abs(theta) >= M_PI / 2 + 0.01) {  // should not be hitting any wall.  //
                                         // todo: get rid of the .05 artifact
                                         // (see #1712)
      valuecheck(max_range, distances(i));
    } else if (theta <= -M_PI / 4) {  // hitting the right wall
      valuecheck(-1.0 / sin(theta), distances(i), tol);
    } else if (theta >= M_PI / 4) {  // hitting the left wall
      valuecheck(1.0 / sin(theta), distances(i), tol);
    } else {  // hitting the front wall
      valuecheck(1 / cos(theta), distances(i), tol);
    }
  }

  auto lcm = make_shared<lcm::LCM>();

  bot_core::planar_lidar_t msg;
  msg.utime = -1;
  msg.nintensities = 0;
  msg.rad0 = min_yaw;
  msg.radstep = (max_yaw - min_yaw) / (distances.size() - 1);

  msg.nranges = distances.size();
  for (int i = 0; i < distances.size(); i++)
    msg.ranges.push_back(distances(i));

  lcm->publish("DRAKE_POINTCLOUD_LIDAR_TEST", &msg);

  // render for debugging:
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(
      lcm, rigid_body_sys->getRigidBodyTree());
  visualizer->output(t, x, u);

  return 0;
}
