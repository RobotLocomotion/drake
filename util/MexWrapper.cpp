#include "MexWrapper.h"

MexWrapper::MexWrapper(std::string const & filename) : m_mex_file(filename)
{	
  m_handle = dlopen(m_mex_file.c_str(), RTLD_NOW);
  if (!m_handle) {
     fprintf(stderr,"%s\n",dlerror());
     return;
  }  

  char* error;
  *(void**) &(m_mexFunc) = dlsym(m_handle, "mexFunction");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr,"%s\n", error);
    return;
  }
}

MexWrapper::~MexWrapper() 
{
  dlclose(m_handle);
}

void MexWrapper::mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) const
{
  m_mexFunc(nlhs, plhs, nrhs, const_cast<const mxArray**>(prhs));
}

std::string MexWrapper::getMexFile() const 
{
  return m_mex_file;
}