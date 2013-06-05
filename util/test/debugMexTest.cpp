
// prints the input

#include <mex.h>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs>0) {
    double v = mxGetScalar(prhs[0]);
    char buf[100];
    mxGetString(prhs[1],buf,100);
    mexPrintf("%f,%s\n",v,buf);
  }
}
