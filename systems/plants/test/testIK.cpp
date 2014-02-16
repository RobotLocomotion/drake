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
  // The state frame of cpp model does not match with the state frame of MATLAB model, since the dofname_to_dofnum is different in cpp and MATLAB
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
  // for(int i = 0;i<model->num_bodies;i++)
  // {
  //   printf("%d %s %5.2f\n",i,model->bodies[i].linkname.c_str(),model->bodies[i].mass);
  // }
  q0 = VectorXd(model->num_dof);
  memcpy(q0.data(),mxGetPr(pxstar),sizeof(double)*model->num_dof);
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
  /*MATFile *presultmat;
  presultmat = matOpen("q_sol.mat","w");
  mxArray* pqsol = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  memcpy(mxGetPr(pqsol),q_sol.data(),sizeof(double)*model->num_dof);
  matPutVariable(presultmat,"q_sol",pqsol);
  matClose(presultmat);*/
  delete com_kc;
  delete[] constraint_array;
  matClose(pnommat);
  return 0;
}
  
