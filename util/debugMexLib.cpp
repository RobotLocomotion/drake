/*
 * libDrakeDebugMex.cpp
 *
 *  Created on: Jun 23, 2013
 *      Author: russt
 */

#include <iostream>
#include <matrix.h>
#include <mat.h>

using namespace std;

mxArray *mxGetProperty(const mxArray *pa, mwIndex index, const char *propname)
{
  return mxGetField(pa,index,propname);
}

extern "C" {
int mexCallMATLAB(int nlhs, mxArray *plhs[], int nrhs,
  mxArray *prhs[], const char *functionName)
{
  cerr << "Invalid mex file: Calls to mexCallMATLAB are not supported by drake_debug_mex" << endl;
  return 0;;
}

mxArray *mexCallMATLABWithTrap(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[], const char *functionName)
{
  cerr << "Invalid mex file: Calls to mexCallMATLAB are not supported by drake_debug_mex" << endl;
  return NULL;
}

}
