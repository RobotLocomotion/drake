#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "../constraint/RigidBodyConstraint.h"
#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyManipulator rbm("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  RigidBodyManipulator* model = &rbm;
  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  Vector2d tspan;
  tspan<<0,1;
  VectorXd q0 = VectorXd::Zero(model->num_positions);
  // The state frame of cpp model does not match with the state frame of MATLAB model, since the dofname_to_dofnum is different in cpp and MATLAB
  q0(3) = 0.8;
  Vector3d com_lb = Vector3d::Zero(); 
  Vector3d com_ub = Vector3d::Zero(); 
  com_lb(2) = 0.9;
  com_ub(2) = 1.0;
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_lb,com_ub,tspan);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(model);
  VectorXd q_sol(model->num_positions);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(model,q0,q0,num_constraints,constraint_array,q_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    return 1;
  }
  VectorXd v = VectorXd::Zero(0);
  model->doKinematicsNew(q_sol, v);
  Vector3d com = model->centerOfMass<double>(0).value();
  printf("%5.2f\n%5.2f\n%5.2f\n",com(0),com(1),com(2));
  /*MATFile *presultmat;
  presultmat = matOpen("q_sol.mat","w");
  mxArray* pqsol = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  memcpy(mxGetPrSafe(pqsol),q_sol.data(),sizeof(double)*model->num_dof);
  matPutVariable(presultmat,"q_sol",pqsol);
  matClose(presultmat);*/
  delete com_kc;
  delete[] constraint_array;
  return 0;
}
  
