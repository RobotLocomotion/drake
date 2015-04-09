#include "MexWrapper.h"
#include <dlfcn.h>

MexWrapper::MexWrapper(std::string const & filename) : m_mex_file(filename)
{	
  m_good = false;

  m_handle = dlopen(m_mex_file.c_str(), RTLD_NOW);
  if (!m_handle) {
     fprintf(stderr,"%s\n",dlerror());
     return;
  }  

  char* error;
  *(void**) &(m_mexFunc) = dlsym(m_handle, "mexFunction");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr,"%s\n", error);
    dlclose(m_handle);
    return;
  } 

  m_good = true;
}

MexWrapper::~MexWrapper() 
{
  if (m_good) {
    dlclose(m_handle);
  }
}

void MexWrapper::mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) const
{
  if (m_good) {
   m_mexFunc(nlhs, plhs, nrhs, const_cast<const mxArray**>(prhs));
  }
}

std::string MexWrapper::getMexFile() const 
{
  return m_mex_file;
}