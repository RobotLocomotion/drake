

#include <iostream>
#include <cstdlib>
#include "drake/systems/plants/RigidBodyTree.h"

using namespace std;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: urdfKinTest urdf_filename" << endl;
    exit(-1);
  }
  RigidBodyTree* model = new RigidBodyTree(argv[1]);
  cout << "=======" << endl;

  // run kinematics with second derivatives 100 times
  Eigen::VectorXd q = model->getZeroConfiguration();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->num_velocities);
  int i;

  if (argc >= 2 + model->num_positions) {
    for (i = 0; i < model->num_positions; i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  // for (i=0; i<model->num_dof; i++)
  //   q(i)=(double)rand() / RAND_MAX;
  KinematicsCache<double> cache = model->doKinematics(q, v);
  //  }

  //  const Vector4d zero(0, 0, 0, 1);
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 1> pt;

  for (i = 0; i < model->bodies.size(); i++) {
    //    model->forwardKin(i, zero, 1, pt);
    auto pt = model->transformPoints(cache, zero, i, 0);
    auto rpy = model->relativeRollPitchYaw(cache, i, 0);
    Eigen::Matrix<double, 6, 1> x;
    x << pt, rpy;
    //    cout << i << ": forward kin: " << model->bodies[i].linkname << " is at
    //    " << pt.transpose() << endl;
    cout << model->bodies[i]->linkname << " ";
    cout << x.transpose() << endl;
    //    for (int j=0; j<pt.size(); j++)
    //        cout << pt(j) << " ";
  }

  auto phi = model->positionConstraints<double>(cache);
  cout << "phi = " << phi.transpose() << endl;

  delete model;
  return 0;
}
