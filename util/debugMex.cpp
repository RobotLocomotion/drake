
/* 
 * To debug mex files as standalone executable (which you can interrogate with gdb, valgrind, etc):
 *   (1) in matlab: use debugMexEval() to write the inputs to file (type 'help debugMexEval' in matlab for details)
 *       This will run as normal, but write all of the inputs to a _mexdebug.mat file.
 *   (2) at the linux terminal: ./debugMex
 *       This will dynamically load your mexfile, open the .mat file, and start calling your mex file with the same inputs. 
 *
 *
 * Happy bugfinding!  - Russ
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>

#include <matrix.h>
#include <mat.h>
#include <string>
#include <map>

using namespace std;

int mexPrintf(const char* message)  // todo: handle the variable arguments case
{
  printf("%s", message);  
}

// todo: implement stubs for other mex functions (mexErrMsgAndTxt, ...) here

typedef struct _dl_data
{
	void* handle;
	void (*mexFunction)(int, mxArray*[], int, const mxArray* []);
} DLData;

int main(int argc, char* argv[])  // todo: take the mex function and dynamically load / run it.
{
	map<string,DLData> mexfiles;
	map<string,DLData>::iterator iter;
//	map<double,void*> mexPointers;

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
    	DLData d;
    	// Dynamically load the mex file:
			d.handle = dlopen(funstr.c_str(),RTLD_LAZY);
			if (!d.handle) {
				fprintf(stderr,"%s\n",dlerror());
				exit(EXIT_FAILURE);
			}
			char* error;
			*(void**) &(d.mexFunction) = dlsym(d.handle,"mexFunction");
			if ((error = dlerror()) != NULL) {
				fprintf(stderr,"%s\n", error);
				exit(EXIT_FAILURE);
			}
			pair<map<string,DLData>::iterator,bool> ret;
			ret = mexfiles.insert(pair<string, DLData>(funstr,d));
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
  for (iter=mexfiles.begin(); iter!=mexfiles.end(); iter++)
  	dlclose(iter->second.handle);

  matClose(pmatfile);
  exit(EXIT_SUCCESS);
}
