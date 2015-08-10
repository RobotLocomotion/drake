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
    mexErrMsgTxt("Drake:quat2expmapSequencemex:Incorrect Usage, [expmap,expmap_dot] = quat2expmapSequencemex(quat,quat_dot)");
  }
  if(mxGetM(prhs[0]) != 4 || mxGetM(prhs[1]) != 4)
  {
    mexErrMsgTxt("Drake:quat2expmapSequencemex:Invalid input, quat and quat_dot must have 4 rows");
  }
  mwSize N = mxGetN(prhs[0]);
  if(mxGetN(prhs[1]) != N)
  {
    mexErrMsgTxt("Drake:quat2expmapSequencemex:Invalid input, quat and quat_dot must have the same number of columns");
  }
  Matrix<double,4,Dynamic> quat(4,N);
  memcpy(quat.data(), mxGetPr(prhs[0]),sizeof(double)*4*N);
  Matrix<double,4,Dynamic> quat_dot(4,N);
  memcpy(quat_dot.data(), mxGetPr(prhs[1]),sizeof(double)*4*N);
  Matrix<double,3,Dynamic> expmap(3,N);
  Matrix<double,3,Dynamic> expmap_dot(3,N);
  quat2expmapSequence(quat,quat_dot,expmap,expmap_dot);
  plhs[0] = eigenToMatlab(expmap);
  plhs[1] = eigenToMatlab(expmap_dot);
}
