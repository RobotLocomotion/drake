#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::VectorXd;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: urdf_manipulator_dynamics_test urdf_filename" << endl;
    exit(-1);
  }
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      argv[1], drake::multibody::joints::kRollPitchYaw, tree.get());

  cout << "=======" << endl;

  // the order of the bodies may be different in matlab, so print it out once
  // here
  cout << tree->bodies.size() << endl;
  for (const auto& body : tree->bodies) {
    cout << body->get_name() << endl;
  }

  VectorXd q = tree->getZeroConfiguration();
  VectorXd v = VectorXd::Zero(tree->get_num_velocities());
  int i;

  if (argc >= 2 + tree->get_num_positions()) {
    for (i = 0; i < tree->get_num_positions(); i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  if (argc >=
      2 + tree->get_num_positions() + tree->get_num_velocities()) {
    for (i = 0; i < tree->get_num_velocities(); i++)
      sscanf(argv[2 + tree->get_num_positions() + i], "%lf", &v(i));
  }

  KinematicsCache<double> cache = tree->doKinematics(q, v);

  auto H = tree->massMatrix(cache);
  cout << H << endl;

  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  auto C = tree->dynamicsBiasTerm(cache, no_external_wrenches);
  cout << C << endl;

  cout << tree->B << endl;

  if (tree->loops.size() > 0) {
    auto phi = tree->positionConstraints(cache);
    cout << phi << endl;
    auto dphi = tree->positionConstraintsJacobian(cache);
    cout << dphi << endl;
  }

  return 0;
}
