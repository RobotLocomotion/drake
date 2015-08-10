#include "mex.h"
#include "drakeMexUtil.h"
#include "drakeGeometryUtil.h"
#include "drakeGradientUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs!=1) {
    mexErrMsgTxt("Incorrect usage: expmap2quatImplmex(expmap)");
  }
  if (mxGetM(prhs[0])!=3 || mxGetN(prhs[0])!= 1) {
    mexErrMsgTxt("expmap should be a 3 x 1 vector\n");
  }
  Map<Vector3d> expmap(mxGetPrSafe(prhs[0]));
  auto ret = expmap2quat(expmap,2);
  if (nlhs >= 1) {
    plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
    memcpy(mxGetPrSafe(plhs[0]), ret.value().data(), sizeof(double)*4);
  }
  if (nlhs >= 2) {
    plhs[1] = mxCreateDoubleMatrix(4,3,mxREAL);
    memcpy(mxGetPrSafe(plhs[1]), ret.gradient().value().data(), 
           sizeof(double)*12);
  }
  if (nlhs >= 3) {
    plhs[2] = mxCreateDoubleMatrix(4,9,mxREAL);
    memcpy(mxGetPrSafe(plhs[2]), ret.gradient().gradient().value().data(), 
           sizeof(double)*36);
  }
}
