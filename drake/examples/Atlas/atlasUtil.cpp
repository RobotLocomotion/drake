#include "drake/examples/Atlas/atlasUtil.h"
#include <stdexcept>
#include <Eigen/Core>
#include "drake/core/Path.h"
#include "drake/util/yaml/yamlUtil.h"

namespace Atlas {
using namespace Eigen;

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

void setupAtlas(std::unique_ptr<RigidBodyTree>& robot, const KinematicModifications& modifications) {
  for (auto it = modifications.attachments.begin(); it != modifications.attachments.end(); ++it) {
    std::shared_ptr<RigidBodyFrame> attach_to_frame = robot->findFrame(it->attach_to_frame);
    if (!attach_to_frame) {
      std::cerr << "frame name: " << it->attach_to_frame << std::endl;
      throw std::runtime_error("Could not find attachment frame when handling urdf modifications");
    }
    robot->addRobotFromURDF(Drake::getDrakePath() + "/" + it->urdf_filename, it->joint_type, attach_to_frame);
  }

  auto filter = [&](const string &group_name) { return modifications.collision_groups_to_keep.find(group_name) == modifications.collision_groups_to_keep.end(); };
  robot->removeCollisionGroupsIf(filter);
  robot->compile();
}

std::unique_ptr<RigidBodyTree> constructAtlas(const std::string& urdf_filename, const KinematicModifications& modifications) {
  std::unique_ptr<RigidBodyTree> robot = std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf_filename));
  setupAtlas(robot, modifications);
  return robot;
}

std::unique_ptr<RigidBodyTree> constructAtlas(const std::string& urdf_filename, const std::string& urdf_modifications_filename) {
  std::unique_ptr<RigidBodyTree> robot = std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf_filename));
  KinematicModifications modifications = parseKinematicModifications(YAML::LoadFile(urdf_modifications_filename));
  setupAtlas(robot, modifications);
  return robot;
}

}