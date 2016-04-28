#include "drake/systems/plants/joints/Joint.h"
#include <iostream>

using namespace Drake;
using namespace Eigen;
using namespace std;

int main() {

  Isometry3d joint_to_parent_body;
  joint_to_parent_body.setIdentity();
  Vector3d axis = Vector3d::Random();

  Joint<double> quaternionFloating("joint1", joint_to_parent_body, std::unique_ptr<JointType<double>>(new QuaternionFloating<double>()));
  Joint<double> rpyFloating("joint2", joint_to_parent_body, std::unique_ptr<JointType<double>>(new RollPitchYawFloating<double>()));
  Joint<double> revolute("joint3", joint_to_parent_body, std::unique_ptr<JointType<double>>(new Revolute<double>(axis)));
  Joint<double> prismatic("joint4", joint_to_parent_body, std::unique_ptr<JointType<double>>(new Prismatic<double>(axis)));
  
  vector<Joint<double>*> joints({&quaternionFloating, &rpyFloating, &revolute, &prismatic});

  default_random_engine generator;
  for (const auto& joint : joints) {
    VectorXd q = joint->randomConfiguration(generator);
    cout << joint->getName() << endl;
    cout << joint->jointTransform(q).matrix() << endl;
    cout << endl;
  }

//  auto q2 = ConvertMatrix<float>()(q);
//  auto& q3 = ConvertMatrix<double>()(q);
//
//  cout << boolalpha << (&q == &q3) << endl;

  return 0;
};
