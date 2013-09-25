#include "mex.h"
#include "drakeUtil.h"
#include "Constraint.h"
#include <cstdlib>

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs<1)
  {
    mexErrMsgIdAndTxt("Drake:deleteConstraintmex:NotEnoughInputs","Usage deleteConstraintmex(constraint_ptr)");
  }
  Constraint* cnst = (Constraint*) getDrakeMexPointer(prhs[0]);

  delete cnst;
}
