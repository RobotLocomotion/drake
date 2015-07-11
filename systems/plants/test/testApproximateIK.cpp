#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "../constraint/RigidBodyConstraint.h"
#include "RigidBodyManipulator.h"
#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>
#include <limits>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyManipulator* model = new RigidBodyManipulator("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  Vector2d tspan;
  tspan<<0,1;
  VectorXd q0 = VectorXd::Zero(model->num_positions);
  q0(3) = 0.8;
  Vector3d com_des = Vector3d::Zero();
  com_des(2) = std::numeric_limits<double>::quiet_NaN();
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_des,com_des);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(model);
  VectorXd q_sol(model->num_positions);
  int info;
  approximateIK(model,q0,q0,num_constraints,constraint_array,q_sol,info,ikoptions);
  printf("INFO = %d\n",info);
  delete com_kc;
  delete[] constraint_array;
  return 0;
}
