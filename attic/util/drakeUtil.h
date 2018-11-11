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

// TODO(siyuan.feng): Cleanup the naming according to the style guide.

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
  DRAKE_ASSERT(source.rows() == 1 || source.cols() == 1);
  if (Size != source.size())
    throw std::runtime_error("Size of source doesn't match destination");
  for (size_t i = 0; i < Size; ++i) {
    destination[i] = static_cast<DestScalar>(source(i));
  }
}

/**
 * Copies the elements of a C array to an Eigen vector (row or column).
 * This function does not resize @p destination, and will throw an exception
 * if dimension mismatches.
 *
 * @param[in] source Fixed sized C array.
 * @param[out] destination Eigen vector
 */
template <typename SourceScalar, size_t Size, typename Derived>
void cArrayToEigenVector(const SourceScalar (&source)[Size],
                         Eigen::MatrixBase<Derived>& destination) {
  DRAKE_ASSERT(destination.rows() == 1 || destination.cols() == 1);
  if (Size != destination.size())
    throw std::runtime_error("Size of source doesn't match destination");
  for (size_t i = 0; i < Size; ++i) {
    destination(i) = static_cast<typename Derived::Scalar>(source[i]);
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

/**
 * Copies the elements of a std::vector to a (row or column) Eigen vector.
 * This function does not resize @p destination, and will throw an exception
 * if dimension mismatches.
 *
 * @param[in] source std vector.
 * @param[out] destination Eigen vector.
 */
template <typename SourceScalar, typename Derived>
void stdVectorToEigenVector(const std::vector<SourceScalar>& source,
                            Eigen::MatrixBase<Derived>& destination) {
  DRAKE_ASSERT(destination.rows() == 1 || destination.cols() == 1);
  if (static_cast<size_t>(destination.size()) != source.size())
    throw std::runtime_error("Size of source doesn't match destination");
  for (size_t i = 0; i < source.size(); ++i) {
    destination(i) = static_cast<typename Derived::Scalar>(source[i]);
  }
}

// note for if/when we split off all Matlab related stuff into a different file:
// this function is not Matlab related
/**
 * Copies the elements of an Eigen Matrix to a std vector of std vectors,
 * s.t. @p destination[i][j] = @p source(i, j).
 * This function resizes @p destination.
 *
 * @param[in] source Eigen matrix
 * @param[out] destination std vector of std vectors
 */
template <typename DestScalar, typename Derived>
void eigenToStdVectorOfStdVectors(
    const Eigen::MatrixBase<Derived>& source,
    std::vector<std::vector<DestScalar>>& destination) {
  destination.resize(source.rows());
  for (Eigen::Index row = 0; row < source.rows(); ++row) {
    auto& destination_row = destination[row];
    destination_row.resize(source.cols());
    for (Eigen::Index col = 0; col < source.cols(); ++col) {
      destination_row[col] = static_cast<DestScalar>(source(row, col));
    }
  }
}

/**
 * Copies the elements of a std vector of std vectors to an Eigen Matrix,
 * s.t. @p destination(i, j) = @p source[i][j].
 * This function does not resize @p destination, and will throw an exception if
 * dimension mismatches.
 *
 * @param[in] source std vector of std vectors
 * @param[out] destination Eigen matrix
 */
template <typename SourceScalar, typename Derived>
void stdVectorOfStdVectorsToEigen(
    const std::vector<std::vector<SourceScalar>>& source,
    Eigen::MatrixBase<Derived>& destination) {
  if (source.size() != static_cast<size_t>(destination.rows()))
    throw std::runtime_error("Size of source doesn't match destination");
  for (size_t row = 0; row < source.size(); ++row) {
    if (source[row].size() != static_cast<size_t>(destination.cols()))
      throw std::runtime_error("Size of source doesn't match destination");
    for (size_t col = 0; col < source[row].size(); ++col) {
      destination(row, col) =
        static_cast<typename Derived::Scalar>(source[row][col]);
    }
  }
}
