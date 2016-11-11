#pragma once

/*
 * drakeUtil.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_stl_types.h"

template <typename Key, typename T>
using eigen_aligned_unordered_map
#ifndef _MSC_VER
    DRAKE_DEPRECATED("Use drake::eigen_aligned_std_unordered_map")
#endif
    = drake::eigen_aligned_std_unordered_map<Key, T>;

template <typename Derived>
inline void sizecheck(const Eigen::MatrixBase<Derived>& mat, size_t rows,
                      size_t cols) {
  if ((mat.rows() != rows) || (mat.cols() != cols))
    throw std::runtime_error(
        "Wrong-sized matrix:  Expected " + std::to_string(rows) + "-by-" +
        std::to_string(cols) + " but got " + std::to_string(mat.rows()) +
        "-by-" + std::to_string(mat.cols()));
}

// note for if/when we split off all Matlab related stuff into a different file:
// this function is not Matlab related
// can only be used when the dimension information of the array is known at
// compile time
template <size_t Rows, size_t Cols, typename Derived>
void eigenToCArrayOfArrays(const Eigen::MatrixBase<Derived>& source,
                           double (&destination)[Rows][Cols]) {
  if (Rows != source.rows())
    throw std::runtime_error(
        "Number of rows of source doesn't match destination");
  if (Cols != source.cols())
    throw std::runtime_error(
        "Number of columns of source doesn't match destination");
  for (size_t row = 0; row < Rows; ++row) {
    for (size_t col = 0; col < Cols; ++col) {
      destination[row][col] = source(row, col);
    }
  }
}

// note for if/when we split off all Matlab related stuff into a different file:
// this function is not Matlab related
/**
 * Copies the elements of a (row or column) Eigen vector to an array.
 *
 * Can only be used when the dimension information of the array is known at
 * compile time.
 * Note that the scalar type of @p source need not match the scalar type of @p
 * destination (@p DestScalar). A static_cast will be performed to convert to
 * elements of @p source to @p DestScalar.
 *
 * @param[in] source Eigen vector source
 * @param[out] destination std::vector destination
 */
template <typename DestScalar, size_t Size, typename Derived>
void eigenVectorToCArray(const Eigen::MatrixBase<Derived>& source,
                         DestScalar (&destination)[Size]) {
  if (Size != source.size())
    throw std::runtime_error("Size of source doesn't match destination");
  for (size_t i = 0; i < Size; ++i) {
    destination[i] = static_cast<DestScalar>(source(i));
  }
}

// note for if/when we split off all Matlab related stuff into a different file:
// this function is not Matlab related
/**
 * Copies the elements of a (row or column) Eigen vector to a std::vector.
 *
 * Note that the scalar type of @p source need not match the scalar type of @p
 * destination (@p DestScalar). A static_cast will be performed to convert to
 * elements of @p source to @p DestScalar.
 *
 * @param[in] source Eigen vector source
 * @param[out] destination std::vector destination
 */
template <typename DestScalar, typename Derived>
void eigenVectorToStdVector(const Eigen::MatrixBase<Derived>& source,
                            std::vector<DestScalar>& destination) {
  DRAKE_ASSERT(source.rows() == 1 || source.cols() == 1);
  destination.resize(static_cast<size_t>(source.size()));
  for (Eigen::Index i = 0; i < source.size(); i++) {
    destination[static_cast<size_t>(i)] = static_cast<DestScalar>(source(i));
  }
}

// note for if/when we split off all Matlab related stuff into a different file:
// this function is not Matlab related
template <typename Derived>
void eigenToStdVectorOfStdVectors(
    const Eigen::MatrixBase<Derived>& source,
    std::vector<std::vector<typename Derived::Scalar> >& destination) {
  destination.resize(source.rows());
  for (Eigen::Index row = 0; row < source.rows(); ++row) {
    auto& destination_row = destination[row];
    destination_row.resize(source.cols());
    for (Eigen::Index col = 0; col < source.cols(); ++col) {
      destination_row[col] = source(row, col);
    }
  }
}

template <typename T>
void addOffset(std::vector<T>& v, const T& offset) {
  std::transform(v.begin(), v.end(), v.begin(),
                 std::bind2nd(std::plus<double>(), offset));
}

void baseZeroToBaseOne(std::vector<int>& vec);

double angleAverage(double theta1, double theta2);

template <typename DerivedTorque, typename DerivedForce, typename DerivedNormal,
          typename DerivedPoint>
std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(
    const Eigen::MatrixBase<DerivedTorque>& torque,
    const Eigen::MatrixBase<DerivedForce>& force,
    const Eigen::MatrixBase<DerivedNormal>& normal,
    const Eigen::MatrixBase<DerivedPoint>& point_on_contact_plane);

