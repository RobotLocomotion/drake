#include "drake/systems/plants/joints/Joint.h"
#include <iostream>

using namespace Drake;
using namespace Eigen;
using namespace std;

int main() {

  Isometry3d joint_to_parent_body;
  joint_to_parent_body.setIdentity();
  Vector3d axis = Vector3d::Random();
  double pitch = 1.0;

  Joint<double> quaternionFloating("quaternionFloating", joint_to_parent_body, std::unique_ptr<JointType<double>>(new QuaternionFloating<double>()));
  Joint<double> rpyFloating("rpyFloating", joint_to_parent_body, std::unique_ptr<JointType<double>>(new RollPitchYawFloating<double>()));
  Joint<double> revolute("revolute", joint_to_parent_body, std::unique_ptr<JointType<double>>(new Revolute<double>(axis)));
  Joint<double> prismatic("prismatic", joint_to_parent_body, std::unique_ptr<JointType<double>>(new Prismatic<double>(axis)));
  Joint<double> helical("helical", joint_to_parent_body, std::unique_ptr<JointType<double>>(new Helical<double>(axis, pitch)));
  Joint<double> fixed("fixed", joint_to_parent_body, std::unique_ptr<JointType<double>>(new Fixed<double>()));
  
  vector<Joint<double>*> joints({&quaternionFloating, &rpyFloating, &revolute, &prismatic, &helical, &fixed});

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
