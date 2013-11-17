#include "mex.h"
#include "RigidBodyConstraint.h"
#include "drakeUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/*
 * [lower_bound,upper_bound] = testPostureConstraintmex(postureConstraint_ptr,t)
 * @param postureConstraint_ptr        A pointer to a PostureConstraint object
 * @param t                            A double scalar, the time to evaluate the lower and upper bounds, This is optional if the constraint is time-invariant
 * @retval lower_bound                 The lower bound of the joints at time t
 * @retval upper_bound                 The upper bound of the joints at time t
 * */
void mexFunction(int nlhs,mxArray* plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 2 && nrhs != 1)
  {
    mexErrMsgIdAndTxt("Drake:testPostureConstraintmex:BadInputs","Usage [lb,ub] = testPostureConstraintmex(pc_ptr,t)");
  }
  double t;
  double* t_ptr;
  if(nrhs == 1)
  {
    t_ptr = NULL;
  }
  else
  {
    t = mxGetScalar(prhs[1]);
    t_ptr = &t;
  }
  PostureConstraint* pc = (PostureConstraint*) getDrakeMexPointer(prhs[0]);
  int nq = pc->getRobotPointer()->num_dof;
  double* lb = new double[nq];
  double* ub = new double[nq];
  pc->bounds(t_ptr,lb,ub);
  plhs[0] = mxCreateDoubleMatrix(nq,1,mxREAL);
  plhs[1] = mxCreateDoubleMatrix(nq,1,mxREAL);
  memcpy(mxGetPr(plhs[0]),lb,sizeof(double)*nq);
  memcpy(mxGetPr(plhs[1]),ub,sizeof(double)*nq);
}
