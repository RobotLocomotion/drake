#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "../constraint/RigidBodyConstraint.h"
#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyTree model("examples/Atlas/urdf/atlas_minimal_contact.urdf");

  Vector2d tspan;
  tspan<<0,1;
  VectorXd q0 = model.getZeroConfiguration();
  // The state frame of cpp model does not match with the state frame of MATLAB model, since the dofname_to_dofnum is different in cpp and MATLAB
  q0(2) = 0.8;
  Vector3d com_lb = Vector3d::Zero(); 
  Vector3d com_ub = Vector3d::Zero(); 
  com_lb(2) = 0.9;
  com_ub(2) = 1.0;
  WorldCoMConstraint com_kc(&model,com_lb,com_ub,tspan);

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back( &com_kc );
  IKoptions ikoptions(&model);
  VectorXd q_sol(model.num_positions);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(&model,q0,q0,constraint_array.size(),constraint_array.data(),q_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    return 1;
  }
  KinematicsCache<double> cache = model.doKinematics(q_sol);
  Vector3d com = model.centerOfMass(cache);
  printf("%5.2f\n%5.2f\n%5.2f\n",com(0),com(1),com(2));
  /*MATFile *presultmat;
  presultmat = matOpen("q_sol.mat","w");
  mxArray* pqsol = mxCreateDoubleMatrix(model.num_dof,1,mxREAL);
  memcpy(mxGetPrSafe(pqsol),q_sol.data(),sizeof(double)model.num_dof);
  matPutVariable(presultmat,"q_sol",pqsol);
  matClose(presultmat);*/
  return 0;
}
  
