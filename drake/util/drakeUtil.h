/*
 * drakeUtil.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

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

template <typename Derived>
inline void sizecheck(const Eigen::MatrixBase<Derived>& mat, size_t rows, size_t cols){
  if ((mat.rows() != rows) || (mat.cols() != cols))
    throw std::runtime_error("Wrong-sized matrix:  Expected " + std::to_string(rows) + "-by-" + std::to_string(cols) + " but got " + std::to_string(mat.rows()) + "-by-" + std::to_string(mat.cols()));
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



template <typename T>
void addOffset(std::vector<T>& v, const T& offset)
{
  std::transform(v.begin(), v.end(), v.begin(), std::bind2nd(std::plus<double>(), offset));
}

DLLEXPORT void baseZeroToBaseOne(std::vector<int>& vec);

DLLEXPORT double angleAverage(double theta1, double theta2);

template <typename DerivedTorque, typename DerivedForce, typename DerivedNormal, typename DerivedPoint>
DLLEXPORT std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(const Eigen::MatrixBase<DerivedTorque> & torque, const Eigen::MatrixBase<DerivedForce> & force, const Eigen::MatrixBase<DerivedNormal> & normal, const Eigen::MatrixBase<DerivedPoint> & point_on_contact_plane);


template <typename DerivedA, typename DerivedB, typename DerivedQ, typename DerivedR, typename DerivedX>
DLLEXPORT void care(Eigen::MatrixBase<DerivedA> const& A, Eigen::MatrixBase<DerivedB> const& B, Eigen::MatrixBase<DerivedQ> const& Q, Eigen::MatrixBase<DerivedR> const& R, Eigen::MatrixBase<DerivedX> & X);

template <typename DerivedA, typename DerivedB, typename DerivedQ, typename DerivedR, typename DerivedK, typename DerivedS>
DLLEXPORT void lqr(Eigen::MatrixBase<DerivedA> const& A, Eigen::MatrixBase<DerivedB> const& B, Eigen::MatrixBase<DerivedQ> const& Q, Eigen::MatrixBase<DerivedR> const& R, Eigen::MatrixBase<DerivedK> & K, Eigen::MatrixBase<DerivedS> & S);

#endif /* DRAKE_UTIL_H_ */
