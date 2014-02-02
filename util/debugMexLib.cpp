/*
 * libDrakeDebugMex.cpp
 *
 *  Created on: Jun 23, 2013
 *      Author: russt
 */

#include <iostream>
#include <vector>

#include <string.h>
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

// todo: keep a list of drake mex pointers and delete them on shutdown
vector<mxArray*> drake_mex_ptrs;

mxArray* createDrakeMexPointer(void* ptr, const char* deleteMethod, const char* name)
{
  mxClassID cid;
  if (sizeof(ptr)==4) cid = mxUINT32_CLASS;
  else if (sizeof(ptr)==8) cid = mxUINT64_CLASS;
  else cerr << "Are you on a 32-bit machine or 64-bit machine??" << endl;
  
  mxArray* mx = mxCreateNumericMatrix(1,1,cid,mxREAL);
  memcpy(mxGetData(mx),&ptr,sizeof(ptr));

  drake_mex_ptrs.push_back(mx);
  return mx;
}

void* getDrakeMexPointer(const mxArray* mx)
{
  void* ptr = NULL;
  memcpy(&ptr,mxGetData(mx),sizeof(ptr));     // note: could use a reinterpret_cast here instead
  return ptr;
}

void cleanupDrakeMexPointers(const mxArray* mx)
{
  cout << "deleting drake mex pointers" << endl;
  for (vector<mxArray*>::iterator iter=drake_mex_ptrs.begin(); iter!=drake_mex_ptrs.end(); iter++) {
    //    delete(getDrakeMexPointer(*iter));
    mxDestroyArray(*iter);
  }
}
