#include "rigidBodyTreeMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  positionDotToVelocityMappingmex(nlhs, plhs, nrhs, prhs);
}
