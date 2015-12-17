#include "rigidBodyTreeMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  forwardJacDotTimesVmex(nlhs, plhs, nrhs, prhs);
}
