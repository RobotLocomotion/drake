/*
 * drake.cpp
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */


bool isa(const mxArray* mxa, const char* class_str)
// mxIsClass seems to not be able to handle derived classes. so i'll implement what I need by calling back to matlab
{
  mxArray* plhs;
  mxArray* prhs[2];
  prhs[0] = const_cast<mxArray*>(mxa);
  prhs[1] = mxCreateString(class_str);
  mexCallMATLAB(1,&plhs,2,prhs,"isa");
  bool tf = (bool) mxGetScalar(plhs);
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
    mexErrMsgAndTxt("Drake:mexCallMATLABsafe:CallbackError", "Error in MATLAB callback.\nSee additional debugging information above");
    mxDestroyArray(ex);
    return true;
  }
  for (i=0; i<nlhs; i++)
    if (!plhs[i]) {
      mexPrintf("Drake mexCallMATLABsafe: error when calling ''%s'' with the following arguments:\n", filename);
      for (i=0; i<nrhs; i++)
        mexCallMATLAB(0,NULL,1,&prhs[i],"disp");
      mexErrMsgAndTxt("Drake:mexCallMATLABsafe:NotEnoughOutputs","Asked for %d outputs, but function only returned %d\n",nrhs,i);
      return true;
    }
  return false;
}



mxArray* constructDrakeMexPointer(void* ptr, void (*delete_fcn)(void*))
{
	mxClassID cid;
	if (sizeof(ptr)==4) cid = mxUINT32_CLASS;
	else if (sizeof(ptr)==8) cid = mxUINT64_CLASS;
  else mexErrMsgIdAndTxt("Drake:constructDrakeMexPointer:PointerSize","Are you on a 32-bit machine or 64-bit machine??");

	int nrhs=1;
	mxArray *prhs[2], *plhs[1];
	mxArray *prhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
  memcpy(mxGetData(prhs[0]),&ptr,sizeof(ptr));

  if (delete_fcn) {
  	nrhs=2;
  	mxArray *prhs[1] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(prhs[1]),&delete_fcn,sizeof(delete_fcn));
  }

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
  memcpy(&ptr,mxGetData(ptrArray),sizeof(ptr));

  return ptr;
}

void destroyDrakeMexPointer(const mxArray* mx)
{
  void* ptr = getDrakeMexPointer(mx);
  void (*delete_fcn)(void*);

	mxArray* ptrArray = mxGetProperty(mx,0,"delete_fcn_ptr");
	if (!ptrArray)
		mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","cannot retrieve 'delete_fcn_ptr' field from this mxArray.  are you sure it's a valid DrakeMexPointer object?");

  if (!mxIsNumeric(ptrArray) || mxGetNumberOfElements(ptrArray)!=1)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","the delete_fcn_ptr property of this DrakeMexPointer does not appear to contain a valid pointer");
  memcpy(&delete_fcn,mxGetData(ptrArray),sizeof(delete_fcn));

  if (!delete_fcn)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","the delete_fcn_ptr property of this DrakeMexPointer does not appear to contain a valid pointer");

  // todo: is there any way to protect against this pointer becoming invalid (e.g. by the original mex function being cleared?)
  delete_fcn(ptr);
}


