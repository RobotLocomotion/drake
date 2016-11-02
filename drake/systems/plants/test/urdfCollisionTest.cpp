#include <iostream>
#include <cstdlib>
#include <vector>

#include "drake/systems/plants/RigidBodyTree.h"

using std::cerr;
using std::cout;
using std::endl;
using std::vector;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: urdfCollisionTest urdf_filename" << endl;
    exit(-1);
  }
  RigidBodyTree* model = new RigidBodyTree(argv[1]);
  if (!model) {
    cerr << "ERROR: Failed to load model from " << argv[1] << endl;
    return -1;
  }

  // run kinematics with second derivatives 100 times
  Eigen::VectorXd q = model->getZeroConfiguration();
  if (argc >= 2 + model->get_num_positions()) {
    for (int i = 0; i < model->get_num_positions(); i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  // for (i=0; i<model->num_dof; i++)
  //   q(i)=(double)rand() / RAND_MAX;
  KinematicsCache<double> cache = model->doKinematics(q);
  //  }

  Eigen::VectorXd phi;
  Eigen::Matrix3Xd normal, xA, xB;
  vector<int> bodyA_idx, bodyB_idx;

  model->collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx);

  cout << "=======" << endl;
  for (int j = 0; j < phi.rows(); ++j) {
    cout << phi(j) << " ";
    for (int i = 0; i < 3; ++i) {
      cout << normal(i, j) << " ";
    }
    for (int i = 0; i < 3; ++i) {
      cout << xA(i, j) << " ";
    }
    for (int i = 0; i < 3; ++i) {
      cout << xB(i, j) << " ";
    }
    cout << model->bodies[bodyA_idx.at(j)]->get_name() << " ";
    cout << model->bodies[bodyB_idx.at(j)]->get_name() << endl;
  }

  delete model;
  return 0;
}
