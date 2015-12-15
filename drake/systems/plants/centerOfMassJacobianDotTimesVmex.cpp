#include "rigidBodyTreeMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  centerOfMassJacobianDotTimesVmex(nlhs, plhs, nrhs, prhs);
}
