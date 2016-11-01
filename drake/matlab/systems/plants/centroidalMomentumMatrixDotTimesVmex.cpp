#include "rigidBodyTreeMexFunctions.h"

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  centroidalMomentumMatrixDotTimesvmex(nlhs, plhs, nrhs, prhs);
}
