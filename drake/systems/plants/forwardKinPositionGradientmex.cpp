#include "rigidBodyTreeMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  forwardKinPositionGradientmex(nlhs, plhs, nrhs, prhs);
}
