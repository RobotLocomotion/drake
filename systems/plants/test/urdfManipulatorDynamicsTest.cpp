
#include <iostream>
#include <Eigen/Dense>
#include "RigidBodyManipulator.h"

using namespace std;

int main(int argc, char* argv[])
{
  if (argc < 2) {
    cerr << "Usage: urdfManipulatorDynamicsTest urdf_filename" << endl;
    exit(-1);
  }
  RigidBodyManipulator* model = new RigidBodyManipulator(argv[1]);
  if (!model) {
    cerr << "ERROR: Failed to load model from " << argv[1] << endl;
    return -1;
  }

  // the order of the bodies may be different in matlab, so print it out once here
  cout << model->num_bodies << endl;
  for (int i = 0; i < model->num_bodies; i++) {
    cout << model->bodies[i]->linkname << endl;
  }

  VectorXd q = VectorXd::Zero(model->num_dof);
  VectorXd v = VectorXd::Zero(model->num_velocities);
  int i;

  if (argc >= 2 + model->num_dof) {
    for (i = 0; i < model->num_dof; i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  if (argc >= 2 + model->num_dof + model->num_velocities) {
    for (i = 0; i < model->num_velocities; i++)
      sscanf(argv[2 + model->num_dof + i], "%lf", &v(i));
  }

  model->doKinematicsNew(q.data(), true, v.data(), true);

  auto H = model->massMatrix<double>();
  cout << H.value() << endl;

  map<int, unique_ptr<GradientVar<double, TWIST_SIZE, 1>> > f_ext;
  auto C = model->inverseDynamics(f_ext);
  cout << C.value() << endl;

  cout << model->B << endl;

  if (model->loops.size()>0) {
    auto phi = model->positionConstraints<double>(1);
    cout << phi.value() << endl;
    cout << phi.gradient().value() << endl;
  }

  delete model;
  return 0;
}
