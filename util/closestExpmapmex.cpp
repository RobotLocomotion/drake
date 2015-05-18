#include "mex.h"
#include "drakeMexUtil.h"
#include "drakeGradientUtil.h"
#include "drakeGeometryUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 2) {
    mexErrMsgIdAndTxt("Drake:closestExpmapmex:InvalidInputs","Usage is closestExpmap(expmap1,expmap2)");
  }
  if (mxGetM(prhs[0]) != 3 || mxGetN(prhs[0]) != 1) {
    mexErrMsgIdAndTxt("Drake:closestExpmapmex:InvalidInputs","expmap1 should be a 3 x 1 vector");
  }
  Map<Vector3d> expmap1(mxGetPr(prhs[0]));
  if (mxGetM(prhs[1]) != 3 || mxGetN(prhs[1]) != 1) {
    mexErrMsgIdAndTxt("Drake:closestExpmapmex:InvalidInputs","expmap2 should be a 3 x 1 vector");
  }
  Map<Vector3d> expmap2(mxGetPr(prhs[1]));
  int gradient_order;
  if (nlhs>1) {
    gradient_order = 1;
  }
  else {
    gradient_order = 0;
  }
  GradientVar<double,3,1> ret = closestExpmap(expmap1, expmap2,gradient_order);
  plhs[0] = eigenToMatlab(ret.value());
  if (nlhs>1) {
    plhs[1] = eigenToMatlab(ret.gradient().value());
  }
}
