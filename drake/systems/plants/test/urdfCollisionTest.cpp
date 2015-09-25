

#include <iostream>
#include <cstdlib>
#include "RigidBodyManipulator.h"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc<2) {
		cerr << "Usage: urdfCollisionTest urdf_filename" << endl;
		exit(-1);
	}
  RigidBodyManipulator* model = new RigidBodyManipulator(argv[1]);
  if (!model) {
  	cerr << "ERROR: Failed to load model from " << argv[1] << endl;
  	return -1;
  }

  // run kinematics with second derivatives 100 times
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model->num_positions);
  int i;

  if (argc>=2+model->num_positions) {
  	for (i=0; i<model->num_positions; i++)
  		sscanf(argv[2+i],"%lf",&q(i));
  }

// for (i=0; i<model->num_dof; i++)
// 	 q(i)=(double)rand() / RAND_MAX;
  KinematicsCache<double> cache = model->doKinematics(q, 0);
//  }

  Eigen::VectorXd phi;
  Eigen::Matrix3Xd normal, xA, xB;
  vector<int> bodyA_idx, bodyB_idx;

  model->collisionDetect(cache, phi,normal,xA,xB,bodyA_idx,bodyB_idx);

  cout << "=======" << endl;
  for (int j=0; j<phi.rows(); ++j) {
    cout << phi(j) << " ";
    for (int i=0; i<3; ++i) {
      cout << normal(i,j) << " ";
    }
    for (int i=0; i<3; ++i) {
      cout << xA(i,j) << " ";
    }
    for (int i=0; i<3; ++i) {
      cout << xB(i,j) << " ";
    }
    cout << model->bodies[bodyA_idx.at(j)]->linkname << " ";
    cout << model->bodies[bodyB_idx.at(j)]->linkname << endl;
  }

  delete model;
  return 0;
}
