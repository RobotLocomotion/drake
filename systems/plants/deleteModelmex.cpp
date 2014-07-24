#include "mex.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:deleteModelmex:NotEnoughInputs","Usage deleteModelmex(model_ptr)");
  }

#ifdef __APPLE__
  return;    // NOTE:  This is autrocious, but until we can resolve the segfaults it will make things usable again on mac.
#endif

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  //mexPrintf("deleting model\n");

  delete model;
}
