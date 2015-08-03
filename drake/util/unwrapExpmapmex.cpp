#include "mex.h"
#include "drakeMexUtil.h"
#include "drakeGeometryUtil.h"
#include "drakeGradientUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 2)
  {
    mexErrMsgTxt("Drake:unwrapExpmapmex:Incorrect Usage, [expmap_unwrap,dexpmap_unwrap] = unwrapExpmapmex(expmap1,expmap2)");
  }
  sizecheck(prhs[0],3,1);
  sizecheck(prhs[1],3,1);
  Map<Vector3d> expmap1(mxGetPr(prhs[0]));
  Map<Vector3d> expmap2(mxGetPr(prhs[1]));
  int gradient_order = nlhs>1?1:0;
  auto ret = unwrapExpmap(expmap1,expmap2,gradient_order);
  plhs[0] = eigenToMatlab(ret.value());
  if(nlhs>1)
  {
    plhs[1] = eigenToMatlab(ret.gradient().value());
  }
}
