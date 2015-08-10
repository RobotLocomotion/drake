#include "mex.h"
#include "drakeMexUtil.h"
#include "drakeGeometryUtil.h"
#include "drakeGradientUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 1)
  {
    mexErrMsgTxt("Drake:flipExpmapmex:Incorrect Usage, [expmap_flip,dexpmap_flip] = flipExpmapmex(expmap)");
  }
  sizecheck(prhs[0],3,1);
  Map<Vector3d> expmap(mxGetPr(prhs[0]));
  int gradient_order = nlhs>1?1:0;
  auto ret = flipExpmap(expmap,gradient_order);
  plhs[0] = eigenToMatlab(ret.value());
  if(nlhs>1)
  {
    plhs[1] = eigenToMatlab(ret.gradient().value());
  }
}
