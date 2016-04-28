#include "drake/systems/plants/joints/Joint.h"
#include <iostream>

using namespace Drake;
using namespace Eigen;
using namespace std;

int main() {

  Isometry3d joint_to_parent_body;
  joint_to_parent_body.setIdentity();

  Joint<double> quaternionFloating("joint1", joint_to_parent_body, std::unique_ptr<JointType<double>>(new QuaternionFloating<double>()));
  Joint<double> rpyFloating("joint2", joint_to_parent_body, std::unique_ptr<JointType<double>>(new RollPitchYawFloating<double>()));

  VectorXd q = VectorXd::Random(7);
  cout << quaternionFloating.getName() << endl;
  cout << quaternionFloating.jointTransform(q).matrix() << endl;

  auto q2 = ConvertMatrix<float>()(q);
  auto& q3 = ConvertMatrix<double>()(q);

  cout << boolalpha << (&q == &q3) << endl;

  return 0;
};
