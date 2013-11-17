#include "mex.h"
#include "drakeUtil.h"
#include "RigidBodyConstraint.h"
#include <cstdlib>

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs<1)
  {
    mexErrMsgIdAndTxt("Drake:deleteRigidBodyConstraintmex:NotEnoughInputs","Usage deleteRigidBodyConstraintmex(constraint_ptr)");
  }
  RigidBodyConstraint* cnst = (RigidBodyConstraint*) getDrakeMexPointer(prhs[0]);

  delete cnst;
}
