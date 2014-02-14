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
  MATFile *pmat;
  pmat = matOpen("../../examples/Atlas/data/atlas_fp.mat","r");
  if(pmat == NULL)
  {
    printf("Error reading mat file\n");
    return(EXIT_FAILURE);
  }
  mxArray* pxstar = matGetVariable(pmat,"xstar");
  if(pxstar == NULL)
  {
    printf("no xstar in mat file\n");
    return(EXIT_FAILURE);
  }
  int nT = 4;
  double* t = new double[nT];
  double dt = 1.0/(nT-1);
  for(int i = 0;i<nT;i++)
  {
    t[i] = dt*i;
  }
  MatrixXd q0(model->num_dof,nT);
  for(int i = 0;i<nT;i++)
  {
    memcpy(q0.data()+model->num_dof*i,mxGetPr(pxstar),sizeof(double)*model->num_dof);
  }
  VectorXd qdot0 = VectorXd::Zero(model->num_dof);
  Vector3d com_des = Vector3d::Zero();
  com_des(2) = nan("");
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_des,com_des);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(model);
  MatrixXd q_sol(model->num_dof,nT);
  MatrixXd qdot_sol(model->num_dof,nT);
  MatrixXd qddot_sol(model->num_dof,nT);
  int info = 0;
  vector<string> infeasible_constraint;
  inverseKinTraj(model,nT,t,qdot0,q0,q0,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  delete com_kc;
  delete[] constraint_array;
  delete[] t;
  matClose(pmat);
  return 0;
}
