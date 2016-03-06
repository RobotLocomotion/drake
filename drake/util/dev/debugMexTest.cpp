
// prints the input

#include <mex.h>
//#include <matrix.h>

#define UNUSED(x) (void)(x)

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  UNUSED(nlhs);
  UNUSED(plhs);

  if (nrhs>0) {
    double v = mxGetScalar(prhs[0]);
    char buf[100];
    mxGetString(prhs[1],buf,100);

    mxArray* C = mxGetProperty(prhs[2],0,"C");
    if (!C) mexErrMsgIdAndTxt("debugMexTest","failed to get property C");

    double* pC = mxGetPrSafe(C);
    
    mexPrintf("%f,%s, C=[%f,%f]\n",v,buf,pC[0],pC[1]);
  }
}
