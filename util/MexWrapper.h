#ifndef MEX_WRAPPER_H
#define MEX_WRAPPER_H

#include "mex.h"
#include <dlfcn.h>
#include <string>

class MexWrapper {
  public:
    MexWrapper(std::string const & filename);
    ~MexWrapper();
    void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) const;
    std::string getMexFile() const;
   private:
   	std::string m_mex_file;
   	void* m_handle;
    void (*m_mexFunc)(int, mxArray*[], int, const mxArray* []);
};

#endif