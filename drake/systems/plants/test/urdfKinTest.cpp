

#include <iostream>
#include <cstdlib>
#include "RigidBodyManipulator.h"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc<2) {
		cerr << "Usage: urdfKinTest urdf_filename" << endl;
		exit(-1);
	}
  RigidBodyManipulator* model = new RigidBodyManipulator(argv[1]);
  if (!model) {
  	cerr << "ERROR: Failed to load model from " << argv[1] << endl;
  	return -1;
  }
  cout << "=======" << endl;

  // run kinematics with second derivatives 100 times
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model->num_positions);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->num_velocities);
  int i;

  if (argc>=2+model->num_positions) {
  	for (i=0; i<model->num_positions; i++)
  		sscanf(argv[2+i],"%lf",&q(i));
  }

// for (i=0; i<model->num_dof; i++)
// 	 q(i)=(double)rand() / RAND_MAX;
  KinematicsCache<double> cache = model->doKinematics(q, v, 0);
//  }

//  const Vector4d zero(0,0,0,1);
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  Eigen::Matrix<double,6,1> pt;

  for (i=0; i<model->bodies.size(); i++) {
//    model->forwardKin(i,zero,1,pt);
		auto pt = model->forwardKin(cache, zero, i, 0, 1, 0);
//    cout << i << ": forward kin: " << model->bodies[i].linkname << " is at " << pt.transpose() << endl;
    cout << model->bodies[i]->linkname << " ";
		cout << pt.value().transpose() << endl;
//    for (int j=0; j<pt.size(); j++)
//    	cout << pt(j) << " ";
  }

  auto phi = model->positionConstraints<double>(cache,1);
  cout << "phi = " << phi.value().transpose() << endl;

  delete model;
  return 0;
}
