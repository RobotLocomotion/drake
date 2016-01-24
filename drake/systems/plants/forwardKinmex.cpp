#include "rigidBodyTreeMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  forwardKinmex(nlhs, plhs, nrhs, prhs);
}
