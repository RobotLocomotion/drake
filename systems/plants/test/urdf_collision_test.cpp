

#include <iostream>
#include <cstdlib>
#include "URDFRigidBodyManipulator.h"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc<2) {
		cerr << "Usage: urdf_collision_test urdf_filename" << endl;
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
  
  VectorXd phi;
  MatrixXd normal, xA, xB;
  vector<int> bodyA_idx, bodyB_idx, bodies_idx;
  
  model->collisionDetect(phi,normal,xA,xB,bodyA_idx,bodyB_idx,bodies_idx);

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
