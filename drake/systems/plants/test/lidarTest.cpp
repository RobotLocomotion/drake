
#include <cmath>
#include "lcmtypes/drake/lcmt_lidar_data.hpp"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  // unit test that sets up a lidar in a box room and verifies the returns
  auto rigid_body_sys = make_shared<RigidBodySystem>(getDrakePath() + "/systems/plants/test/lidarTest.sdf",DrakeJoint::FIXED);

  double t = 0;
  VectorXd x = VectorXd::Zero(0);
  VectorXd u = VectorXd::Zero(0);

//  rigid_body_sys->getRigidBodyTree()->drawKinematicTree("/tmp/lidar.dot");

  auto lcm = make_shared<lcm::LCM>();

  auto distances = rigid_body_sys->output(t,x,u);

  // these parameters must match the sdf (i've also implicitly hard-coded the box geometry from the sdf)
  const double min_yaw = -1.7;
  const double max_yaw = 1.7;
  const double tol = 0.05; // todo: improve the tolerance (see #1712)

  for (int i=0; i<distances.size(); i++) {
    auto const distance = distances(i);
    double theta = min_yaw + (max_yaw-min_yaw)*i/(distances.size()-1);
    if (abs(theta)>=M_PI/2+.05) { // should not be hitting any wall.  // todo: get rid of the .05 artifact (see #1712)
      valuecheck(-1.0,distance);
    } else if (theta<=-M_PI/4) { // hitting the right wall
      valuecheck(-1.0/sin(theta),distance,tol);
    } else if (theta>=M_PI/4) { // hitting the left wall
      valuecheck(1.0/sin(theta),distance,tol);
    } else { // hitting the front wall
      valuecheck(1/cos(theta),distance,tol);
    }

    drake::lcmt_lidar_field range;
    range.id = drake::lcmt_lidar_data::RANGE;
    range.num_values = 1;
    range.values.push_back(distance);

    drake::lcmt_lidar_field position;
    position.id = drake::lcmt_lidar_data::POSITION;
    position.num_values = 3;
    if (distance < 0.0)
    {
      position.values.push_back(nan(""));
      position.values.push_back(nan(""));
      position.values.push_back(nan(""));
    }
    else
    {
      position.values.push_back(distance * cos(theta) - 1.0);
      position.values.push_back(distance * sin(theta));
      position.values.push_back(0.0);
    }

    drake::lcmt_lidar_data msg;
    msg.timestamp = 0;
    msg.scan_angle = theta;
    msg.scan_direction = false;
    msg.fields.push_back(range);
    msg.fields.push_back(position);
    msg.num_fields = msg.fields.size();

    lcm->publish("DRAKE_POINTCLOUD_LIDAR_TEST", &msg);
  }

  // render for debugging:
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,rigid_body_sys->getRigidBodyTree());
  visualizer->output(t,x,u);

  return 0;
}
