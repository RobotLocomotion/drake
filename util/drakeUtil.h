/*
 * drakeUtil.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include "mex.h"
#include <stdexcept>
#include <vector>
#include <utility>
#include <Eigen/Core>

#ifndef DRAKE_UTIL_H_
#define DRAKE_UTIL_H_

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

// Helper routines
DLLEXPORT bool isa(const mxArray* mxa, const char* class_str);
DLLEXPORT bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename);

// Mex pointers shared through matlab
DLLEXPORT mxArray* createDrakeMexPointer(void* ptr, const char* name="", int num_additional_inputs=0, mxArray *delete_fcn_additional_inputs[] = NULL, const char* subclass_name=NULL);  // increments lock count
// Note: the same mex function which calls this method will be called with the syntax mexFunction(drake_mex_ptr) as the destructor

DLLEXPORT void* getDrakeMexPointer(const mxArray* mx);

template <typename Derived> inline void destroyDrakeMexPointer(const mxArray* mx)
{
  Derived typed_ptr = (Derived) getDrakeMexPointer(mx);

  //mexPrintf("deleting drake mex pointer\n");
  delete typed_ptr;
  mexUnlock();  // decrement lock count

//  mexPrintf(mexIsLocked() ? "mex file is locked\n" : "mex file is unlocked\n");
}

template <typename Derived>
DLLEXPORT mxArray* eigenToMatlabSparse(Eigen::MatrixBase<Derived> const & M, int & num_non_zero);

template <typename DerivedA>
mxArray* eigenToMatlab(const DerivedA &m)
{
  // this avoids zero initialization that would occur using mxCreateDoubleMatrix with nonzero dimensions.
  // see https://classes.soe.ucsc.edu/ee264/Fall11/cmex.pdf, page 8
  mxArray* pm = mxCreateDoubleMatrix(0, 0, mxREAL);
  int rows = static_cast<int>(m.rows());
  int cols = static_cast<int>(m.cols());
  int numel = rows * cols;
  mxSetM(pm, rows);
  mxSetN(pm, cols);
  if (numel)
    mxSetData(pm, mxMalloc(sizeof(double) * numel));
  memcpy(mxGetPr(pm), m.data(), sizeof(double)* numel);
  return pm;
}

template<int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> matlabToEigen(const mxArray* matlab_array)
{
  const mwSize* size_array = mxGetDimensions(matlab_array);
  Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> ret(size_array[0], size_array[1]);
  memcpy(ret.data(), mxGetPr(matlab_array), sizeof(double) * ret.size());
  return ret;
}

template<int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Map<const Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime>> matlabToEigenMap(const mxArray* matlab_array)
{
  Eigen::Map<const Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime>> ret(mxGetPr(matlab_array), mxGetM(matlab_array), mxGetN(matlab_array));
  return ret;
}

DLLEXPORT std::string mxGetStdString(const mxArray* array);

template <typename Scalar>
mxArray* stdVectorToMatlab(const std::vector<Scalar>& vec) {
  mxArray* pm = mxCreateDoubleMatrix(static_cast<int>(vec.size()), 1, mxREAL);
  for (int i = 0; i < static_cast<int>(vec.size()); i++) {
    mxGetPr(pm)[i] = (double) vec[i];
  }
  return pm;
}

DLLEXPORT void sizecheck(const mxArray* mat, int M, int N);

template <size_t Rows, size_t Cols>
void matlabToCArrayOfArrays(const mxArray *source, double (&destination)[Rows][Cols])  {
  // Matlab arrays come in as column-major data. The format used in e.g. LCM messages is an array of arrays.
  // from http://stackoverflow.com/a/17569578/2228557
  sizecheck(source, static_cast<int>(Rows), static_cast<int>(Cols));
  double* source_data = mxGetPr(source);
  for (size_t row = 0; row < Rows; ++row) {
    for (size_t col = 0; col < Cols; ++col) {
      destination[row][col] = source_data[row + col * Rows];
    }
  }
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
// can only be used when the dimension information of the array is known at compile time
template <size_t Rows, size_t Cols, typename Derived>
void eigenToCArrayOfArrays(const Eigen::MatrixBase<Derived>& source, double (&destination)[Rows][Cols]) {
  if (Rows != source.rows())
    throw std::runtime_error("Number of rows of source doesn't match destination");
  if (Cols != source.cols())
    throw std::runtime_error("Number of columns of source doesn't match destination");
  for (size_t row = 0; row < Rows; ++row) {
    for (size_t col = 0; col < Cols; ++col) {
      destination[row][col] = source(row, col);
    }
  }
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
// can only be used when the dimension information of the array is known at compile time
template <size_t Size, typename Derived>
void eigenVectorToCArray(const Eigen::MatrixBase<Derived>& source, double (&destination)[Size]) {
  if (Size != source.size())
    throw std::runtime_error("Size of source doesn't match destination");
  for (size_t i = 0; i < Size; ++i) {
    destination[i] = source(i);
  }
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
template <typename Derived>
void eigenVectorToStdVector(const Eigen::MatrixBase<Derived>& source, std::vector<typename Derived::Scalar>& destination) {
  assert(source.rows() == 1 || source.cols() == 1);
  destination.resize(static_cast<size_t>(source.size()));
  for (Eigen::DenseIndex i = 0; i < source.size(); i++) {
    destination[static_cast<size_t>(i)] = source(i);
  }
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
template <typename Derived>
void eigenToStdVectorOfStdVectors(const Eigen::MatrixBase<Derived>& source, std::vector< std::vector<typename Derived::Scalar> >& destination) {
  destination.resize(source.rows());
  for (Eigen::DenseIndex row = 0; row < source.rows(); ++row) {
    auto& destination_row = destination[row];
    destination_row.resize(source.cols());
    for (Eigen::DenseIndex col = 0; col < source.cols(); ++col) {
      destination_row[col] = source(row, col);
    }
  }
}


DLLEXPORT mwSize sub2ind(mwSize ndims, const mwSize* dims, const mwSize* sub);

template <typename T>
void addOffset(std::vector<T>& v, const T& offset)
{
  std::transform(v.begin(), v.end(), v.begin(), std::bind2nd(std::plus<double>(), offset));
}

DLLEXPORT void baseZeroToBaseOne(std::vector<int>& vec);

DLLEXPORT double angleAverage(double theta1, double theta2);

DLLEXPORT std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(Eigen::Vector3d torque, Eigen::Vector3d force, Eigen::Vector3d normal, Eigen::Vector3d point_on_contact_plane);
template <typename T>
const std::vector<T> matlabToStdVector(const mxArray* in);

DLLEXPORT double *mxGetPrSafe(const mxArray *pobj);

DLLEXPORT mxArray* mxGetPropertySafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldSafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetPropertySafe(const mxArray* array, size_t index, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldSafe(const mxArray* array, size_t index, std::string const& field_name);
DLLEXPORT void mxSetFieldSafe(mxArray* array, size_t index, std::string const & fieldname, mxArray* data);
DLLEXPORT mxArray* mxGetFieldOrPropertySafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldOrPropertySafe(const mxArray* array, size_t index, std::string const& field_name);


#endif /* DRAKE_UTIL_H_ */
