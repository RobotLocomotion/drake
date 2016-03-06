#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "../constraint/RigidBodyConstraint.h"
#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>
#include <limits>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyTree rbm("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  Vector2d tspan;
  tspan<<0,1;
  int nT = 3;
  double t[3]={0.0,0.5,1.0};
  MatrixXd q0(rbm.num_positions, nT);
  for(int i = 0;i<nT;i++)
  {
    q0.col(i) = rbm.getZeroConfiguration();
    q0(3,i) = 0.8;
  }
  Vector3d com_des = Vector3d::Zero();
  com_des(2) = std::numeric_limits<double>::quiet_NaN();
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(&rbm,com_des,com_des);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(&rbm);
  MatrixXd q_sol(rbm.num_positions,nT);
  int* info = new int[nT];
  vector<string> infeasible_constraint;
  inverseKinPointwise(&rbm,nT,t,q0,q0,num_constraints,constraint_array,q_sol,info,infeasible_constraint,ikoptions);
  for(int i = 0;i<nT;i++)
  {
    printf("INFO[%d] = %d ",i,info[i]);
    if(info[i]!= 1)
    {
      return 1;
    }
  }
  printf("\n");
  delete com_kc;
  delete[] constraint_array;
  delete[] info;
  return 0;
}
