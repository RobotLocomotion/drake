#include "drake/matlab/util/mex_wrapper.h"

#include <cstdio>
#include <utility>

#include <dlfcn.h>

MexWrapper::MexWrapper(std::string filename) : m_mex_file(std::move(filename)) {
  m_good = false;

  // load the mexfile
  m_handle = dlopen(m_mex_file.c_str(), RTLD_NOW);

  if (m_handle == nullptr) {
    std::fprintf(stderr, "%s\n", dlerror());
    return;
  }

  // locate and store the entry point in a function pointer
  char* error;
  *reinterpret_cast<void**>(&(m_mexFunc)) = dlsym(m_handle, "mexFunction");

  if ((error = dlerror()) != nullptr) {
    fprintf(stderr, "%s\n", error);
    dlclose(m_handle);
    return;
  }

  m_good = true;
}

MexWrapper::~MexWrapper() {
  if (m_good) {
    dlclose(m_handle);
  }
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
