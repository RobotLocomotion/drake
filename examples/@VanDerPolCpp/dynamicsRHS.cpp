#include "mex.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "drakeUtil.h"

using namespace Eigen;
using namespace std;
/*
 * c++ version of
 *     function xdot = dynamicsRHS(~,~,x,~)
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) 
{
  Map<Vector2d> x(mxGetPrSafe(prhs[2]));
  plhs[0] = mxCreateDoubleMatrix(2,1,mxREAL);
  Map<Vector2d> xdot(mxGetPr(plhs[0]));

  xdot << x(1), -x(0)-x(1)*(x(0)*x(0)-1);
}