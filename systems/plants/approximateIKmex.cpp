#include "RigidBodyIK.h"
#include <mex.h>
#include "RigidBodyManipulator.h"
#include "constraint/RigidBodyConstraint.h"
#include "drakeUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs<5)
  {
    mexErrMsgIdAndTxt("Drake:approximateIKmex:NotEnoughInputs","Usage approximateIKmex(model_ptr,q_seed,q_nom,constraint1,constraint2,...,ikoptions)");
  }
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  int nq = model->num_dof;
  Map<VectorXd> q_seed(mxGetPr(prhs[1]),nq);
  Map<VectorXd> q_nom(mxGetPr(prhs[2]),nq);
  mexPrintf("get q_seed,q_nom\n");
  int num_constraints = nrhs-4;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  for(int i = 0;i<num_constraints;i++)
  {
    constraint_array[i] = (RigidBodyConstraint*) getDrakeMexPointer(prhs[3+i]);
  }
  mexPrintf("get constraint\n");
  IKoptions* ikoptions = (IKoptions*) getDrakeMexPointer(prhs[nrhs-1]);
  mexPrintf("get ikoptions\n");
  plhs[0] = mxCreateDoubleMatrix(nq,1,mxREAL);
  Map<VectorXd> q_sol(mxGetPr(plhs[0]),nq);
  int info;
  approximateIK(model,q_seed,q_nom,num_constraints,constraint_array,q_sol,info,*ikoptions);
  mexPrintf("finish approximateIK\n"); 
  if (nlhs>1) 
  {
    plhs[1] = mxCreateDoubleScalar((double) info);
  }
  delete[] constraint_array;
}
