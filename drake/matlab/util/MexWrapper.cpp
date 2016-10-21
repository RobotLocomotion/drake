#include "drake/matlab/util/MexWrapper.h"
#if defined(WIN32) || defined(WIN64)
#else
#include <dlfcn.h>
#endif

MexWrapper::MexWrapper(std::string const& filename) : m_mex_file(filename) {
  m_good = false;

#if defined(WIN32) || defined(WIN64)
#else
  // load the mexfile
  m_handle = dlopen(m_mex_file.c_str(), RTLD_NOW);
  if (!m_handle) {
    fprintf(stderr, "%s\n", dlerror());
    return;
  }

  // locate and store the entry point in a function pointer
  char* error;
  *(void**)&(m_mexFunc) = dlsym(m_handle, "mexFunction");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr, "%s\n", error);
    dlclose(m_handle);
    return;
  }

  m_good = true;
#endif
}

MexWrapper::~MexWrapper() {
#if defined(WIN32) || defined(WIN64)
#else
  if (m_good) {
    dlclose(m_handle);
  }
#endif
}

// the caller is responsible for allocating all of the mxArray* memory and
// freeing it when
//
void MexWrapper::mexFunction(int nlhs, mxArray* plhs[], int nrhs,
                             const mxArray* prhs[]) const {
  if (m_good) {
    m_mexFunc(nlhs, plhs, nrhs, const_cast<const mxArray**>(prhs));
  }
}

std::string MexWrapper::getMexFile() const { return m_mex_file; }
