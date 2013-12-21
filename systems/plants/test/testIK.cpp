#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "../constraint/RigidBodyConstraint.h"
#include "URDFRigidBodyManipulator.h"
#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>
#include "mat.h"

using namespace std;
using namespace Eigen;
int main()
{
  URDFRigidBodyManipulator* model = loadURDFfromFile("../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  Vector2d tspan;
  tspan<<0,1;
  VectorXd q0;
  MATFile *pnommat;
  pnommat = matOpen("../../examples/Atlas/data/atlas_fp.mat","r");
  if(pnommat == NULL)
  {
    printf("Error reading mat file\n");
    return(EXIT_FAILURE);
  }
  mxArray* pxstar = matGetVariable(pnommat,"xstar");
  if(pxstar == NULL)
  {
    printf("no xstar in mat file\n");
    return(EXIT_FAILURE);
  }
  q0 = VectorXd(model->num_dof);
  memcpy(q0.data(),mxGetPr(pxstar),sizeof(double)*model->num_dof);
  mxArray* pq0 = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  memcpy(mxGetPr(pq0),q0.data(),sizeof(double)*model->num_dof);
  model->doKinematics(q0.data());
  Vector3d com0;
  model->getCOM(com0);
  printf("%4.2f %4.2f %4.2f\n",com0(0),com0(1),com0(2));
  mxArray* pcom0 = mxCreateDoubleMatrix(3,1,mxREAL);
  memcpy(mxGetPr(pcom0),com0.data(),sizeof(double)*3);
  mxArray* pjoint_lb = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  mxArray* pjoint_ub = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  memcpy(mxGetPr(pjoint_lb),model->joint_limit_min,sizeof(double)*model->num_dof);
  memcpy(mxGetPr(pjoint_ub),model->joint_limit_max,sizeof(double)*model->num_dof);
  Vector3d com_lb = Vector3d::Zero(); 
  Vector3d com_ub = Vector3d::Zero(); 
  com_lb(2) = 0.9;
  com_ub(2) = 1.0;
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_lb,com_ub,tspan);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(model);
  VectorXd q_sol(model->num_dof);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(model,q0,q0,num_constraints,constraint_array,q_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  MATFile *presultmat;
  presultmat = matOpen("q_sol.mat","w");
  mxArray* pqsol = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  memcpy(mxGetPr(pqsol),q_sol.data(),sizeof(double)*model->num_dof);
  matPutVariable(presultmat,"q_sol",pqsol);
  matPutVariable(presultmat,"com0",pcom0);
  matPutVariable(presultmat,"q_nom",pq0);
  matPutVariable(presultmat,"joint_limit_min",pjoint_lb);
  matPutVariable(presultmat,"joint_limit_max",pjoint_ub);
  matClose(presultmat);
  delete com_kc;
  delete[] constraint_array;
  matClose(pnommat);
  return 0;
}
  
