

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

  // run kinematics with second derivatives 100 times
  VectorXd q = VectorXd::Zero(model->num_positions);
  int i;

  if (argc>=2+model->num_positions) {
  	for (i=0; i<model->num_positions; i++)
  		sscanf(argv[2+i],"%lf",&q(i));
  }

// for (i=0; i<model->num_dof; i++)
// 	 q(i)=(double)rand() / RAND_MAX;
    model->doKinematics(q,false);
//  }

//  const Vector4d zero(0,0,0,1);
  Vector3d zero = Vector3d::Zero();
  Matrix<double,6,1> pt;

  for (i=0; i<model->num_bodies; i++) {
//    model->forwardKinNew(i,zero,1,pt);
		auto pt = model->forwardKinNew(zero, i, 0, 1, 0);
//    cout << i << ": forward kin: " << model->bodies[i].linkname << " is at " << pt.transpose() << endl;
    cout << model->bodies[i]->linkname << " ";
		cout << pt.value().transpose() << endl;
//    for (int j=0; j<pt.size(); j++)
//    	cout << pt(j) << " ";
  }

  delete model;
  return 0;
}
