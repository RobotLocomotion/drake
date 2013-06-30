
#include <mex.h>
#include "drakeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs<1)
    mexErrMsgIdAndTxt("Drake:deleteMexPointer:BadInputs","Usage: deleteMexPointer(DrakeMexPointer_obj)");

  destroyDrakeMexPointer(prhs[0]);
}
