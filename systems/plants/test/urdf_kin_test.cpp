
#include <iostream>
#include <cstdlib>
#include "../urdf.h"

using namespace std;

int main(int argc, char* argv[])
{
  // todo: pull urdf filename off the command line
  std::string filename("../../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
  RigidBodyManipulator* model = loadURDF(filename);
  
  // run kinematics with second derivatives 100 times
  double q[34];
  int i;
  
  for (int n=0; n<20; n++) {
    for (i=0; i<34; i++)  
      q[i]=(double)rand() / RAND_MAX;
    model->doKinematics(q,true);
  }
  
  Vector4d zero;
  zero << 0,0,0,1; 
  
  for (i=0; i<=model->NB; i++) {
    cout << "forward kin: " << model->bodies[i].linkname << " is at " << model->forwardKin(i,zero,1).transpose() << endl;
  } 
  
  delete model;
  return 0;
}
