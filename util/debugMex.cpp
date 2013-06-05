
/* 
 * To debug mex files as standalone executable (which you can interrogate with gdb, valgrind, etc):
 *   (1) in matlab: use debugMexEval() to write the inputs to file (type 'help debugMexEval' in matlab for details)
 *       This will run as normal, but write all of the inputs to a _mexdebug.mat file.
 *   (2) at the linux terminal: ./debugMex your_mex_filename_without_extension.
 *       This will dynamically load your mexfile, open the .mat file, and start calling your mex file with the same inputs. 
 * Happy bugfinding!  - Russ
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>

#include <matrix.h>
#include <mat.h>
#include <string>

using namespace std;

int mexPrintf(const char* message)  // todo: handle the variable arguments case
{
  printf("%s", message);  
}

// todo: implement stubs for other mex functions (mexErrMsgAndTxt, ...) here


int main(int argc, char* argv[])  // todo: take the mex function and dynamically load / run it.
{
  if (argc<2) fprintf(stderr,"Usage: debugMex mex_filename_without_extension"); 
  
  string mexfile(argv[1]);

  // Dynamically load the mex file:
  void* handle = dlopen((mexfile+".mexmaci64").c_str(),RTLD_LAZY);
  if (!handle) {
    fprintf(stderr,"%s\n",dlerror());
    exit(EXIT_FAILURE);
  }
  void (*mexFunction)(int, mxArray*[], int, const mxArray *[]);
  char* error;
  *(void **) (&mexFunction) = dlsym(handle,"mexFunction");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr,"%s\n", error);
    exit(EXIT_FAILURE);
  }
  
  // load inputs from matfile
  MATFile* pmatfile = matOpen((mexfile+"_mexdebug.mat").c_str(), "r");
  if (!pmatfile) {
    fprintf(stderr,"Failed to open %s\n", (mexfile+"_mexdebug.mat").c_str());
    exit(EXIT_FAILURE);
  }
  
  int i, count=1;  char buff[100], countstr[6] = "00000";
  string name;
  
  while(true) {
    sprintf(buff,"%d",count);
    memcpy(&countstr[5-strlen(buff)],buff,strlen(buff));
    
    name = "varargin_"; name += countstr;
    mxArray* varargin = matGetVariable(pmatfile, name.c_str());
    if (!varargin) break;  // then we've gotten to the end of the input "tape"
    
    name = "nrhs_"; name += countstr;
    int nrhs = mxGetScalar(matGetVariable(pmatfile, name.c_str()));

    name = "nlhs_"; name += countstr;
    int nlhs = mxGetScalar(matGetVariable(pmatfile, name.c_str()));
    
    mxArray** plhs = new mxArray*[nlhs];
    mxArray** prhs = new mxArray*[nrhs];
    for (i=0; i<nrhs; i++) 
      prhs[i] = mxGetCell(varargin,i);
    
    printf("input %s\n",countstr);
    
    (*mexFunction)(nlhs,plhs,nrhs,const_cast<const mxArray**>(prhs));

    mxDestroyArray(varargin);
    for (i=0; i<nlhs; i++) 
      mxDestroyArray(plhs[i]); 
    
    delete[] plhs;
    
    count++;
  }
  
  // cleanup          
  matClose(pmatfile);
  dlclose(handle);
  exit(EXIT_SUCCESS);
}