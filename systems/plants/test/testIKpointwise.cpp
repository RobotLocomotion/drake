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
  int nT = 3;
  double t[3]={0.0,0.5,1.0};
  MatrixXd q0(model->num_dof,nT);
  for(int i = 0;i<nT;i++)
  {
    memcpy(q0.data()+model->num_dof*i,mxGetPr(pxstar),sizeof(double)*model->num_dof);
  }
  Vector3d com_des = Vector3d::Zero();
  com_des(2) = nan("");
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_des,com_des);
  int num_constraints = 1;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  IKoptions ikoptions(model);
  MatrixXd q_sol(model->num_dof,nT);
  int* info = new int[nT];
  vector<string> infeasible_constraint;
  inverseKinPointwise(model,nT,t,q0,q0,num_constraints,constraint_array,q_sol,info,infeasible_constraint,ikoptions);
  for(int i = 0;i<nT;i++)
  {
    printf("INFO[%d] = %d ",i,info[i]);
  }
  printf("\n");
  delete com_kc;
  delete[] constraint_array;
  delete[] info;
  matClose(pmat);
  return 0;
}
