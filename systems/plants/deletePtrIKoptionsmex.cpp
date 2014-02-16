#include "mex.h"
#include "drakeUtil.h"
#include "IKoptions.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs<1)
  {
    mexErrMsgIdAndTxt("drake:deletePtrIKoptionsmex:NotEnoughInputs","Usage deletePtrIKoptionsmex(ikoptions_ptr)");
  }
  IKoptions* ikoptions = (IKoptions*) getDrakeMexPointer(prhs[0]);

  delete ikoptions;
}
