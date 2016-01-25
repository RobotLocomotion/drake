
#include <iostream>
#include <Eigen/Dense>
#include "drake/systems/plants/RigidBodyTree.h"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
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

  // the order of the bodies may be different in matlab, so print it out once here
  cout << model->bodies.size() << endl;
  for (int i = 0; i < model->bodies.size(); i++) {
    cout << model->bodies[i]->linkname << endl;
  }

  VectorXd q = model->getZeroConfiguration();
  VectorXd v = VectorXd::Zero(model->num_velocities);
  int i;

  if (argc >= 2 + model->num_positions) {
    for (i = 0; i < model->num_positions; i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  if (argc >= 2 + model->num_positions + model->num_velocities) {
    for (i = 0; i < model->num_velocities; i++)
      sscanf(argv[2 + model->num_positions + i], "%lf", &v(i));
  }

  KinematicsCache<double> cache = model->doKinematics(q, v);

  auto H = model->massMatrix(cache);
  cout << H << endl;

  eigen_aligned_unordered_map<RigidBody const *, Matrix<double, TWIST_SIZE, 1> > f_ext;
  auto C = model->dynamicsBiasTerm(cache, f_ext);
  cout << C << endl;

  cout << model->B << endl;

  if (model->loops.size()>0) {
    auto phi = model->positionConstraints(cache);
    cout << phi << endl;
    auto dphi = model->positionConstraintsJacobian(cache);
    cout << dphi << endl;
  }

  return 0;
}
