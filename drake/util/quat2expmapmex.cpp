#include "mex.h"
#include "drakeGeometryUtil.h"
#include "drakeMexUtil.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 1)
  {
    mexErrMsgTxt("Incorrect usage, quat2expmapmex(q)");
  }
  Map<Vector4d> q(mxGetPrSafe(prhs[0]));
  int gradient_order = nlhs==2?1:0;
  GradientVar<double,3,1> ret = quat2expmap(q,gradient_order);
  plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
  memcpy(mxGetPrSafe(plhs[0]),ret.value().data(),sizeof(double)*3);
  if(nlhs>1)
  {
    plhs[1] = mxCreateDoubleMatrix(3,4,mxREAL);
    memcpy(mxGetPrSafe(plhs[1]),ret.gradient().value().data(),sizeof(double)*12);
  }
}
