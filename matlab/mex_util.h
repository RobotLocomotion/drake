#pragma once

#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <mex.h>

#include "drake/common/autodiff.h"

DLL_EXPORT_SYM bool isa(const mxArray* mxa, const char* class_str);

DLL_EXPORT_SYM bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs,
                                      mxArray* prhs[], const char* filename);

DLL_EXPORT_SYM double* mxGetPrSafe(const mxArray* pobj);

DLL_EXPORT_SYM mxArray* mxGetPropertySafe(const mxArray* array,
                                          std::string const& field_name);

DLL_EXPORT_SYM mxArray* mxGetFieldSafe(const mxArray* array,
                                       std::string const& field_name);

DLL_EXPORT_SYM mxArray* mxGetPropertySafe(const mxArray* array, size_t index,
                                          std::string const& field_name);

DLL_EXPORT_SYM mxArray* mxGetFieldSafe(const mxArray* array, size_t index,
                                       std::string const& field_name);

DLL_EXPORT_SYM void mxSetFieldSafe(mxArray* array, size_t index,
                                   std::string const& fieldname, mxArray* data);

DLL_EXPORT_SYM mxArray* mxGetFieldOrPropertySafe(const mxArray* array,
                                                 std::string const& field_name);

DLL_EXPORT_SYM mxArray* mxGetFieldOrPropertySafe(const mxArray* array,
                                                 size_t index,
                                                 std::string const& field_name);

// MEX pointers shared through MATLAB. Note that the same MEX function which
// calls this method will be called with the syntax mexFunction(drake_mex_ptr)
// as the destructor.

DLL_EXPORT_SYM mxArray* createDrakeMexPointer(
    void* ptr, const std::string& name = "", int type_id = -1,
    int num_additional_inputs = 0,
    mxArray* delete_fcn_additional_inputs[] = nullptr,
    const std::string& subclass_name = "",
    const std::string& mex_function_name_prefix = "");  // Increment lock count.

DLL_EXPORT_SYM void* getDrakeMexPointer(const mxArray* mx);

template <typename Derived>
inline void destroyDrakeMexPointer(const mxArray* mx) {
  if (!isa(mx, "DrakeMexPointer")) {
    mexErrMsgIdAndTxt("Drake:destroyDrakeMexPointer:BadInputs",
                      "This object is not a DrakeMexPointer. Delete failed.");
  }

  Derived typed_ptr = reinterpret_cast<Derived>(getDrakeMexPointer(mx));
  delete typed_ptr;
  mexUnlock();  // Decrement lock count.
}

template <typename Derived>
DLL_EXPORT_SYM mxArray* eigenToMatlabSparse(Eigen::MatrixBase<Derived> const& M,
                                            int* num_non_zero);

template <typename DerivedA>
mxArray* eigenToMatlab(const DerivedA& m) {
  // This avoids zero initialization that would occur using
  // mxCreateDoubleMatrix with nonzero dimensions, see
  // https://classes.soe.ucsc.edu/ee264/Fall11/cmex.pdf, page 8.
  mxArray* pm = mxCreateDoubleMatrix(0, 0, mxREAL);
  const int rows = static_cast<int>(m.rows());
  const int cols = static_cast<int>(m.cols());
  const int numel = rows * cols;
  mxSetM(pm, rows);
  mxSetN(pm, cols);

  if (numel) {
    mxSetData(pm, mxMalloc(sizeof(double) * numel));
  }

  std::memcpy(mxGetPr(pm), m.data(), sizeof(double) * numel);
  return pm;
}

template <int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> matlabToEigen(
    const mxArray* matlab_array) {
  const mwSize* size_array = mxGetDimensions(matlab_array);
  Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> ret(
      size_array[0], size_array[1]);
  std::memcpy(ret.data(), mxGetPr(matlab_array), sizeof(double) * ret.size());
  return ret;
}

template <int Rows, int Cols>
Eigen::Map<const Eigen::Matrix<double, Rows, Cols>> matlabToEigenMap(
    const mxArray* mex) {
  Eigen::Index rows, cols;  // At runtime.
  if (mxIsEmpty(mex)) {
    // Be lenient when it comes to dimensions in the empty input case.
    if (Rows == Eigen::Dynamic && Cols == Eigen::Dynamic) {
      // If both dimensions are dynamic, then follow the dimensions of the
      // MATLAB matrix.
      rows = mxGetM(mex);
      cols = mxGetN(mex);
    } else {
      // If only one dimension is dynamic, use the known dimension at compile
      // time and set the other dimension to zero.
      rows = Rows == Eigen::Dynamic ? 0 : Rows;
      cols = Cols == Eigen::Dynamic ? 0 : Cols;
    }
  } else {
    // The non-empty case.
    rows = Rows == Eigen::Dynamic ? mxGetM(mex) : Rows;
    cols = Cols == Eigen::Dynamic ? mxGetN(mex) : Cols;
  }

  if (Rows != Eigen::Dynamic && Rows != rows) {
    std::ostringstream stream;
    stream << "Error converting MATLAB matrix. Expected " << Rows
           << " rows, but got " << mxGetM(mex) << ".";
    throw std::runtime_error(stream.str().c_str());
  }

  if (Cols != Eigen::Dynamic && Cols != cols) {
    std::ostringstream stream;
    stream << "Error converting MATLAB matrix. Expected " << Cols
           << " cols, but got " << mxGetN(mex) << ".";
    throw std::runtime_error(stream.str().c_str());
  }

  const double* data = rows * cols == 0 ? nullptr : mxGetPrSafe(mex);
  return Eigen::Map<const Eigen::Matrix<double, Rows, Cols>>(data, rows, cols);
}

DLL_EXPORT_SYM Eigen::SparseMatrix<double> matlabToEigenSparse(
    const mxArray* mex);

DLL_EXPORT_SYM std::string mxGetStdString(const mxArray* array);
DLL_EXPORT_SYM std::vector<std::string> mxGetVectorOfStdStrings(
    const mxArray* array);

template <typename Scalar>
mxArray* stdVectorToMatlab(const std::vector<Scalar>& vec) {
  mxArray* pm = mxCreateDoubleMatrix(static_cast<int>(vec.size()), 1, mxREAL);

  for (int i = 0; i < static_cast<int>(vec.size()); i++) {
    mxGetPr(pm)[i] = static_cast<double>(vec[i]);
  }

  return pm;
}

DLL_EXPORT_SYM mxArray* stdStringToMatlab(const std::string& str);
DLL_EXPORT_SYM mxArray* vectorOfStdStringsToMatlab(
    const std::vector<std::string>& strs);

DLL_EXPORT_SYM void sizecheck(const mxArray* mat, mwSize M, mwSize N);

template <size_t Rows, size_t Cols>
void matlabToCArrayOfArrays(const mxArray* source,
                            double (&destination)[Rows][Cols]) {
  // MATLAB arrays come in as column-major data. The format used in e.g., LCM
  // messages is an array of arrays.
  // From http://stackoverflow.com/a/17569578/2228557.
  sizecheck(source, static_cast<int>(Rows), static_cast<int>(Cols));
  double* source_data = mxGetPr(source);

  for (size_t row = 0; row < Rows; ++row) {
    for (size_t col = 0; col < Cols; ++col) {
      destination[row][col] = source_data[row + col * Rows];
    }
  }
}

DLL_EXPORT_SYM mwSize sub2ind(mwSize ndims, const mwSize* dims,
                              const mwSize* sub);

template <typename T>
const std::vector<T> matlabToStdVector(const mxArray* in);

template <int RowsAtCompileTime, int ColsAtCompileTime, typename DerType>
mxArray* eigenToMatlabGeneral(
    const Eigen::MatrixBase<Eigen::Matrix<
        Eigen::AutoDiffScalar<DerType>, RowsAtCompileTime, ColsAtCompileTime>>&
        mat) {
  return eigenToTaylorVar(mat);
}

template <int RowsAtCompileTime, int ColsAtCompileTime>
mxArray* eigenToMatlabGeneral(
    const Eigen::MatrixBase<
        Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime>>& mat) {
  return eigenToMatlab(mat.const_cast_derived());
}
