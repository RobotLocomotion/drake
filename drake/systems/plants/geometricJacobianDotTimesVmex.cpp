#include "rigidBodyManipulatorMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  geometricJacobianDotTimesVmex(nlhs, plhs, nrhs, prhs);
}
