#include "mex.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:deleteModelmex:NotEnoughInputs","Usage deleteModelMex(model_ptr)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  delete model;
}
