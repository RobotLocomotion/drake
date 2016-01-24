#include "rigidBodyTreeMexFunctions.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  centroidalMomentumMatrixDotTimesvmex(nlhs, plhs, nrhs, prhs);
}
