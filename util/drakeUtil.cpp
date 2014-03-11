/*
 * drakeUtil.cpp
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include <mex.h>
#include <string.h>
#include <string>

using namespace std;

bool isa(const mxArray* mxa, const char* class_str)
// mxIsClass seems to not be able to handle derived classes. so i'll implement what I need by calling back to matlab
{
  mxArray* plhs;
  mxArray* prhs[2];
  prhs[0] = const_cast<mxArray*>(mxa);
  prhs[1] = mxCreateString(class_str);
  mexCallMATLAB(1,&plhs,2,prhs,"isa");
  bool tf = (mxGetScalar(plhs)==0.0);
  mxDestroyArray(plhs);
  mxDestroyArray(prhs[1]);
  return tf;
}

bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename)
{
  int i;
  mxArray* ex = mexCallMATLABWithTrap(nlhs,plhs,nrhs,prhs,filename);
  if (ex) {
    mexPrintf("DrakeSystem S-Function: error when calling ''%s'' with the following arguments:\n",filename);
    for (i=0; i<nrhs; i++)
      mexCallMATLAB(0,NULL,1,&prhs[i],"disp");
    mxArray *report;
    mexCallMATLAB(1,&report,1,&ex,"getReport");
    char *errmsg = mxArrayToString(report);
    mexPrintf(errmsg);
    mxFree(errmsg);
    mxDestroyArray(report);
    mexErrMsgIdAndTxt("Drake:mexCallMATLABsafe:CallbackError", "Error in MATLAB callback.\nSee additional debugging information above");
    mxDestroyArray(ex);
    return true;
  }
  for (i=0; i<nlhs; i++)
    if (!plhs[i]) {
      mexPrintf("Drake mexCallMATLABsafe: error when calling ''%s'' with the following arguments:\n", filename);
      for (i=0; i<nrhs; i++)
        mexCallMATLAB(0,NULL,1,&prhs[i],"disp");
      mexErrMsgIdAndTxt("Drake:mexCallMATLABsafe:NotEnoughOutputs","Asked for %d outputs, but function only returned %d\n",nrhs,i);
      return true;
    }
  return false;
}



mxArray* createDrakeMexPointer(void* ptr, const char* deleteMethod, const char* name)
{
	mxClassID cid;
	if (sizeof(ptr)==4) cid = mxUINT32_CLASS;
	else if (sizeof(ptr)==8) cid = mxUINT64_CLASS;
  else mexErrMsgIdAndTxt("Drake:constructDrakeMexPointer:PointerSize","Are you on a 32-bit machine or 64-bit machine??");

	const int nrhs=3;
	mxArray *prhs[nrhs], *plhs[1];

	prhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
  memcpy(mxGetData(prhs[0]),&ptr,sizeof(ptr));

	prhs[1] = mxCreateString(deleteMethod);

  prhs[2] = mxCreateString(name);

  // call matlab to construct mex pointer object
  mexCallMATLABsafe(1,plhs,nrhs,prhs,"DrakeMexPointer");

  return plhs[0];
}

void* getDrakeMexPointer(const mxArray* mx)
{
	void* ptr = NULL;

	// todo: optimize this by caching the pointer values, as described in
	// http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=1590
	mxArray* ptrArray = mxGetProperty(mx,0,"ptr");
	if (!ptrArray)
		mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","cannot retrieve 'ptr' field from this mxArray.  are you sure it's a valid DrakeMexPointer object?");

  if (!mxIsNumeric(ptrArray) || mxGetNumberOfElements(ptrArray)!=1)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","the ptr property of this DrakeMexPointer does not appear to contain a valid pointer");
  memcpy(&ptr,mxGetData(ptrArray),sizeof(ptr));     // note: could use a reinterpret_cast here instead

  return ptr;
}

