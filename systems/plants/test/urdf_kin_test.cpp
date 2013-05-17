
#include <iostream>
#include <cstdlib>
#include "urdf.h"

using namespace std;

int main(int argc, char* argv[])
{
  // todo: pull urdf filename off the command line
  URDFRigidBodyManipulator* model = loadURDFfromFile("../../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
//  URDFRigidBodyManipulator* model = loadURDFfromFile("/Users/russt/drc/software/models/mit_gazebo_models/mit_robot/model.urdf");
  if (!model) return -1;
  
  // run kinematics with second derivatives 100 times
  VectorXd q(model->num_dof,1);
  int i;
  
  for (int n=0; n<20; n++) {
    for (i=0; i<model->num_dof; i++)
      q(i)=(double)rand() / RAND_MAX;
    model->doKinematics(q.data(),true);
  }
  
  const Vector4d zero(0,0,0,1);
  Vector3d pt;
  
  for (i=0; i<=model->num_bodies; i++) {
    model->forwardKin(i,zero,1,pt);
    cout << "forward kin: " << model->bodies[i].linkname << " is at " << pt << endl;
  } 
  
  delete model;
  return 0;
}
