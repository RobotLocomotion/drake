#include <iostream>
#include "../RigidBody.h"
#include "../RevoluteJoint.h"
#include "../PrismaticJoint.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <cstdlib>
#include <random>

namespace e = Eigen;

int main(int argc, char **argv)
{
  RigidBody body;
  auto transform_to_parent_body = e::AffineCompact3d::Identity();
//  e::Matrix<double, TWIST_SIZE, RevoluteJoint::NUM_VELOCITIES> joint_axis;
//  joint_axis << 1, 0, 0, 0, 0, 0;
//  RevoluteJoint* joint = new RevoluteJoint(body, transform_to_parent_body, joint_axis);
//  e::Vector3d joint_axis = Vector3d::Random();
//  joint_axis.normalize();
  e::Vector3d joint_axis;
  joint_axis << 0.0, 0.0, 1.0;

//  std::default_random_engine generator;
//  std::normal_distribution<double> distribution;
//  double pitch = distribution(generator);
  RevoluteJoint joint(body, transform_to_parent_body, joint_axis);

  RevoluteJoint::MotionSubspaceType S;
  RevoluteJoint::DMotionSubspaceType dS;
//  e::Matrix<double, TWIST_SIZE, RevoluteJoint::NUM_VELOCITIES> S;
  double q[1];
  q[0] = 1.0;

  joint.motionSubspace(q, S, &dS);
  std::cout << S << std::endl;
  std::cout << std::endl;

  std::cout << dS << std::endl;
  std::cout << std::endl;

  std::cout << joint.getTransformToParentBody().matrix() << std::endl;
  std::cout << std::endl;

  std::cout << joint.jointTransform(q).matrix() << std::endl;

  return 0;
}
