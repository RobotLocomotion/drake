#include <iostream>
#include "RigidBody.h"
#include "RevoluteJoint.h"
#include "PrismaticJoint.h"
#include "HelicalJoint.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <cstdlib>
#include <random>

namespace e = Eigen;

int main(int argc, char **argv)
{
  RigidBody body;
  auto transform_to_parent_body = e::AffineCompact3d::Identity();
  e::Vector3d joint_axis;
  joint_axis << 0.0, 0.0, 1.0;

  std::default_random_engine generator;
  std::normal_distribution<double> distribution;
  double pitch = distribution(generator);
//  DrakeJoint* joint_ptr = new RevoluteJoint(body, transform_to_parent_body, joint_axis);
  DrakeJoint* joint_ptr = new HelicalJoint(body, transform_to_parent_body, joint_axis, pitch);

  double q[1];
  q[0] = 1.0;

  DrakeJoint::MotionSubspaceType S;
  e::MatrixXd dS;
  joint_ptr->motionSubspace(q, S, &dS);
  std::cout << S << std::endl;
  std::cout << std::endl;

  std::cout << dS << std::endl;
  std::cout << std::endl;

  std::cout << joint_ptr->getTransformToParentBody().matrix() << std::endl;
  std::cout << std::endl;

  std::cout << joint_ptr->jointTransform(q).matrix() << std::endl;

  delete joint_ptr;

  return 0;
}
