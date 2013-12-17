#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "../constraint/RigidBodyConstraint.h"
#include "URDFRigidBodyManipulator.h"
#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>

using namespace std;
using namespace Eigen;
int main()
{
  URDFRigidBodyManipulator* model = loadURDFfromFile("../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
  printf("lb ");
  for(int i = 0;i<model->num_dof;i++)
  {
    printf("%5.3f ",model->joint_limit_min[i]);
    model->joint_limit_min[i] = 0.0;
  }
  printf("\n");
  printf("ub ");
  for(int i = 0;i<model->num_dof;i++)
  {
    printf("%5.3f ",model->joint_limit_max[i]);
    model->joint_limit_max[i] = 0.0;
  }
  printf("\n");
  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  Vector2d tspan;
  tspan<<0,1;
  VectorXd q0(model->num_dof);
  q0 = VectorXd::Zero(model->num_dof);
  Vector3d com_des = Vector3d::Zero();
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_des,com_des);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(model);
  VectorXd q_sol(model->num_dof);
  int info;
  approximateIK(model,q0,q0,num_constraints,constraint_array,q_sol,info,ikoptions);
  delete com_kc;
  delete[] constraint_array;
  return 0;
}
