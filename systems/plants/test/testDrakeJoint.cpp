#include <iostream>
#include <random>

#include "RigidBody.h"
#include "RevoluteJoint.h"
#include "PrismaticJoint.h"
#include "HelicalJoint.h"
#include "QuaternionFloatingJoint.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <climits>
#include <ctime>
//#include <cstdlib>


using namespace Eigen;

int main(int argc, char **argv)
{
  RigidBody body;
  auto transform_to_parent_body = Isometry3d::Identity();

  std::default_random_engine generator;
  generator.seed(time(0));
  std::normal_distribution<double> distribution;
  double pitch = distribution(generator);

  std::vector<std::unique_ptr<DrakeJoint>> joints;
  joints.push_back(std::unique_ptr<DrakeJoint>(new HelicalJoint("helicalJoint", body, transform_to_parent_body, Vector3d::Random(), pitch)));
  RevoluteJoint* revolute_joint = new RevoluteJoint("revoluteJoint", body, transform_to_parent_body, Vector3d::Random());
  revolute_joint->setJointLimits(-1.0, std::numeric_limits<double>::infinity());
  joints.push_back(std::unique_ptr<DrakeJoint>(revolute_joint));
  joints.push_back(std::unique_ptr<DrakeJoint>(new PrismaticJoint("prismaticJoint", body, transform_to_parent_body, Vector3d::Random())));
  joints.push_back(std::unique_ptr<DrakeJoint>(new QuaternionFloatingJoint("quaternionFloatingJoint", body, transform_to_parent_body)));

  for (const auto& joint : joints)
  {
    double q[joint->getNumPositions()];

    joint->randomConfiguration(q, generator);

    std::cout << "Joint name: " << joint->getName() << std::endl;

    std::cout << "q: " << std::endl;
    for (int i = 0; i < joint->getNumPositions(); ++i) {
      std::cout << q[i] << ", ";
    }
    std::cout << std::endl;


    DrakeJoint::MotionSubspaceType S;
    MatrixXd dS;

    joint->motionSubspace(q, S, &dS);
    std::cout << "motion subspace:" << std::endl << S << std::endl << std::endl;

    std::cout << "motion subspace gradient:" << std::endl << dS << std::endl << std::endl;

    std::cout << "transform to parent body:" << std::endl << joint->getTransformToParentBody().matrix() << std::endl << std::endl;

    std::cout << "joint transform:" << std::endl << joint->jointTransform(q).matrix() << std::endl << std::endl;

    std::cout << std::endl;
  }

  return 0;
}
