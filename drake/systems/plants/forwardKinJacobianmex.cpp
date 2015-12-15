#include "rigidBodyTreeMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  forwardKinJacobianmex(nlhs, plhs, nrhs, prhs);
}
