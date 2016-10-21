#pragma once

#include <mex.h>

#include <string>

class DLL_EXPORT_SYM MexWrapper {
 public:
  explicit MexWrapper(std::string const& filename);
  ~MexWrapper();
  void mexFunction(int nlhs, mxArray* plhs[], int nrhs,
                   const mxArray* prhs[]) const;
  std::string getMexFile() const;

 private:
  std::string m_mex_file;
  bool m_good;
  void* m_handle;
  void (*m_mexFunc)(int, mxArray* [], int, const mxArray* []);
};
