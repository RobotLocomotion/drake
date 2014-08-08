

#include <iostream>
#include <cstdlib>
#include "URDFRigidBodyManipulator.h"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc<2) {
		cerr << "Usage: urdf_kin_test urdf_filename" << endl;
		exit(-1);
	}
  URDFRigidBodyManipulator* model = loadURDFfromFile(argv[1]);
//  URDFRigidBodyManipulator* model = loadURDFfromFile("/Users/russt/drc/software/models/mit_gazebo_models/mit_robot/model.urdf");
  if (!model) {
  	cerr << "ERROR: Failed to load model from " << argv[1] << endl;
  	return -1;
  }
  
  // run kinematics with second derivatives 100 times
  VectorXd q = VectorXd::Zero(model->num_dof);
  int i;

  if (argc>=2+model->num_dof) {
  	for (i=0; i<model->num_dof; i++)
  		sscanf(argv[2+i],"%lf",&q(i));
  }
  
// for (i=0; i<model->num_dof; i++)
// 	 q(i)=(double)rand() / RAND_MAX;
    model->doKinematics(q.data(),false);
//  }
  
  const Vector4d zero(0,0,0,1);
//  Vector3d pt;
  Matrix<double,6,1> pt;
  
  for (i=0; i<model->num_bodies; i++) {
    model->forwardKin(i,zero,1,pt);
//    cout << i << ": forward kin: " << model->bodies[i].linkname << " is at " << pt.transpose() << endl;
    cout << model->bodies[i]->linkname << " ";
    for (int j=0; j<pt.size(); j++)
    	cout << pt(j) << " ";
    cout << endl;
  } 
  
  delete model;
  return 0;
}
