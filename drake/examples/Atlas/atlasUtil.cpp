#include "drake/examples/Atlas/atlasUtil.h"
#include <stdexcept>
#include <Eigen/Core>
#include "drake/util/yaml/yamlUtil.h"

using namespace Eigen;
using drake::Side;

bool ankleCloseToLimits(double akx, double aky, double tol)
{
  if (tol<0) {
    throw std::runtime_error("tol should be non-negative in ankleCloseToLimits");
  }
  Matrix<double, 8, 2> A; 
  Matrix<double, 8, 1> b;
  A << 0.5044,   -0.8635,   
       0.1059,   -0.9944, 
       1.0000,    0.0000,
       -0.1083,   -0.9941,
       -0.5044,   -0.8635,
       0.4510,    0.8925,
       -1.0000,   -0.0000,
       -0.4555,    0.8902;
  b << 1.0253,    1.0137,    0.6411,    1.0143,    1.0253,    0.6163,    0.6411,    0.6183;
  Vector2d ankle;
  ankle << akx, aky;
  return ((A*ankle-b).array() > -tol).any();
}

std::unique_ptr<RigidBodyTree> constructAtlas(std::unique_ptr<RigidBodyTree> robot, const RobotPropertyCache& rpc, const AtlasKinematicOptions& options=AtlasKinematicOptions()) {

  for (auto it = options.hands.begin(); it != options.hands.end(); ++it) {
    Side side = it->first;
    AtlasHand hand_type = it->second;
    std::shared_ptr<RigidBody> hand_body = robot->bodies[rpc.hand_ids.at(side)];
    switch (hand_type) {
      case AtlasHand::NONE:
        break;
      case AtlasHand::ROBOTIQ:
        robot->addRobotFromURDF(getDrakePath() + "/examples/Atlas/urdf/robotiq.urdf", DrakeJoint::FIXED, hand_body);
        break;
      case AtlasHand::ROBOTIQ_WEIGHT:
        robot->addRobotFromURDF(getDrakePath() + "/examples/Atlas/urdf/robotiq_box.urdf", DrakeJoint::FIXED, hand_body);
        break;
      default:
        std::cerr << "WARNING: unrecognized hand type: " << hand_type << std::endl;
    }
  }

  auto filter = [&](const string &group_name) { return options.collision_groups_to_keep.find(group_name) == options.collision_groups_to_keep.end(); };
  robot->removeCollisionGroupsIf(filter);
  robot->compile();

  return robot;
}

std::unique_ptr<RigidBodyTree> constructAtlasV5(const std::string& urdf_filename, const std::string& control_config_filename, const AtlasKinematicOptions& options=AtlasKinematicOptions()) {
  std::unique_ptr<RigidBodyTree> robot = std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf_filename));
  RobotPropertyCache rpc = parseKinematicTreeMetadata(control_config_filename, *robot);
  return constructAtlasV5(robot, rpc, options);
}
