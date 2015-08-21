#ifndef MEX_WRAPPER_H
#define MEX_WRAPPER_H

#include "mex.h"
#include <string>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeMexUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

class DLLEXPORT MexWrapper {
  public:
    MexWrapper(std::string const & filename);
    ~MexWrapper();
    void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) const;
    std::string getMexFile() const;
   private:
   	std::string m_mex_file;
   	bool m_good;
   	void* m_handle;
    void (*m_mexFunc)(int, mxArray*[], int, const mxArray* []);
};

#endif