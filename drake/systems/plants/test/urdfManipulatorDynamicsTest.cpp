#include <iostream>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::VectorXd;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: urdfManipulatorDynamicsTest urdf_filename" << endl;
    exit(-1);
  }
  auto model = std::unique_ptr<RigidBodyTree>(new RigidBodyTree(argv[1]));
  if (!model) {
    cerr << "ERROR: Failed to load model from " << argv[1] << endl;
    return -1;
  }
  cout << "=======" << endl;

  // the order of the bodies may be different in matlab, so print it out once
  // here
  cout << model->bodies.size() << endl;
  for (const auto& body : model->bodies) {
    cout << body->get_name() << endl;
  }

  VectorXd q = model->getZeroConfiguration();
  VectorXd v = VectorXd::Zero(model->get_num_velocities());
  int i;

  if (argc >= 2 + model->get_num_positions()) {
    for (i = 0; i < model->get_num_positions(); i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  if (argc >=
      2 + model->get_num_positions() + model->get_num_velocities()) {
    for (i = 0; i < model->get_num_velocities(); i++)
      sscanf(argv[2 + model->get_num_positions() + i], "%lf", &v(i));
  }

  KinematicsCache<double> cache = model->doKinematics(q, v);

  auto H = model->massMatrix(cache);
  cout << H << endl;

  const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;
  auto C = model->dynamicsBiasTerm(cache, no_external_wrenches);
  cout << C << endl;

  cout << model->B << endl;

  if (model->loops.size() > 0) {
    auto phi = model->positionConstraints(cache);
    cout << phi << endl;
    auto dphi = model->positionConstraintsJacobian(cache);
    cout << dphi << endl;
  }

  return 0;
}
