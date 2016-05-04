#include "drake/multibody_dynamics/joints/Joints.h"
#include <iostream>
#include <memory>

using namespace Eigen;
using namespace drake;

int main() {
  std::vector<std::unique_ptr<Joint<double>>> joints;
  Vector3d axis = Vector3d::Random();
  double pitch = 1.0;

  Transform3D<double> tranform_to_parent = Transform3D<double>::Identity();
  joints.push_back(std::move(std::unique_ptr<Joint<double>>(new QuaternionFloatingJoint<double>("quaternion", tranform_to_parent))));
  joints.push_back(std::move(std::unique_ptr<Joint<double>>(new RollPitchYawFloatingJoint<double>("rollpitchyaw", tranform_to_parent))));
  joints.push_back(std::move(std::unique_ptr<Joint<double>>(new FixedJoint<double>("fixed", tranform_to_parent))));
  joints.push_back(std::move(std::unique_ptr<Joint<double>>(new RevoluteJoint<double>("revolute", tranform_to_parent, axis))));
  joints.push_back(std::move(std::unique_ptr<Joint<double>>(new PrismaticJoint<double>("prismatic", tranform_to_parent, axis))));
  joints.push_back(std::move(std::unique_ptr<Joint<double>>(new HelicalJoint<double>("helical", tranform_to_parent, axis, pitch))));

  std::default_random_engine generator;
  for (const auto& joint_ptr : joints) {
    auto q = joint_ptr->randomConfiguration(generator);
    std::cout << joint_ptr->jointTransform(q).matrix() << std::endl << std::endl;
  }


  return 0;
};
