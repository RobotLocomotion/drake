
#include <iostream>
#include <cstdlib>
#include <urdf.h>

using namespace std;

int main(int argc, char* argv[])
{
  // todo: pull urdf filename off the command line
  URDFRigidBodyManipulator* model = loadURDFfromFile("../../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
//  URDFRigidBodyManipulator* model = loadURDFfromFile("/Users/russt/drc/software/models/mit_gazebo_models/mit_robot/model.urdf");
  
  // run kinematics with second derivatives 100 times
  double q[34];
  int i;
  
  for (int n=0; n<20; n++) {
    for (i=0; i<34; i++)  
      q[i]=(double)rand() / RAND_MAX;
    model->doKinematics(q,true);
  }
  
  const Vector4d zero(0,0,0,1);
  Vector3d pt;
  
  for (i=0; i<=model->NB; i++) {
    model->forwardKin(i,zero,1,pt);
    cout << "forward kin: " << model->bodies[i].linkname << " is at " << pt << endl;
  } 
  
  delete model;
  return 0;
}
