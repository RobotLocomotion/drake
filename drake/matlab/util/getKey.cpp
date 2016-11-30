#include <mex.h>

#ifdef WIN32
#include <windows.h>
#else
#include <stdio.h>
#endif

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 1) {
    mexPrintf("Usage: getKey(vkey)\n");
    return;
  }

  plhs[0] =
      mxCreateLogicalScalar(GetAsyncKeyState((int)mxGetScalar(prhs[0])) != 0);
}
