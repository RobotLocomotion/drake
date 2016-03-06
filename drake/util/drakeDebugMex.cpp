
/* 
 * Read the documentation in debugMexEval.m for instructions.
 * Happy bugfinding!  - Russ
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MexWrapper.h"

#include <matrix.h>
#include <mat.h>
#include <string>
#include <map>

using namespace std;

void cleanupDrakeMexPointers(void)
{
  // intentionally left blank; overloaded in debugMexLib.cpp using a DYLD_INSERT_LIBRARIES
}

int mexPrintf(const char* message)  // todo: handle the variable arguments case
{
  return printf("%s", message);  
}

mxArray *mexCallMATLABWithTrap(int nlhs, mxArray *plhs[], int nrhs,
   mxArray *prhs[], const char *functionName)
{
	printf("called mexCallMATLABWithTrap.\n");
	return NULL;
}

// todo: implement stubs for other mex functions (mexErrMsgAndTxt, ...) here

int main(int argc, char* argv[])  // todo: take the mex function and dynamically load / run it.
{
	map<string,MexWrapper> mexfiles;
	map<string,MexWrapper>::iterator iter;

  // load inputs from matfile
  MATFile* pmatfile = matOpen("/tmp/mex_debug.mat", "r");
  if (!pmatfile) {
    fprintf(stderr,"Failed to open %s\n", "/tmp/mex_debug.mat");
    exit(EXIT_FAILURE);
  }

  int i, count=1;  char buff[100], countstr[6] = "00000";
  string name;

  while(true) {
    sprintf(buff,"%d",count);
    memcpy(&countstr[5-strlen(buff)],buff,strlen(buff));

    name = "fun_"; name += countstr;
    mxArray* fun = matGetVariable(pmatfile, name.c_str());
    if (!fun) break;  // then we've gotten to the end of the input "tape"

    char* str = mxArrayToString(fun);
    string funstr(str);
    mxFree(str);

    iter=mexfiles.find(funstr);
    if (iter == mexfiles.end()) {
    	MexWrapper mex = MexWrapper(funstr);
			pair<map<string,MexWrapper>::iterator,bool> ret;
			ret = mexfiles.insert(pair<string, MexWrapper>(funstr,mex));
			iter = ret.first;
    }

    name = "varargin_"; name += countstr;
    mxArray* varargin = matGetVariable(pmatfile, name.c_str());
    
    name = "nrhs_"; name += countstr;
    int nrhs = mxGetScalar(matGetVariable(pmatfile, name.c_str()));

    name = "nlhs_"; name += countstr;
    int nlhs = mxGetScalar(matGetVariable(pmatfile, name.c_str()));
    
    mxArray** plhs = new mxArray*[nlhs];
    mxArray** prhs = new mxArray*[nrhs];
    for (i=0; i<nrhs; i++) 
      prhs[i] = mxGetCell(varargin,i);
    
    printf("%s: %s\n",countstr,funstr.c_str());
    
    iter->second.mexFunction(nlhs,plhs,nrhs,const_cast<const mxArray**>(prhs));

    mxDestroyArray(varargin);
    for (i=0; i<nlhs; i++) 
      mxDestroyArray(plhs[i]); 
    
    delete[] plhs;
    
    count++;
  }
  
  // cleanup          
  cleanupDrakeMexPointers();

  matClose(pmatfile);
  exit(EXIT_SUCCESS);
}
