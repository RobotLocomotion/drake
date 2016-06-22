/// @file
/// This file contains abbreviated definitions for certain specializations of
/// Eigen::Matrix that are commonly used in Drake.

#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

namespace drake {

/// A column vector of size 3, templated on scalar type.
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

/// A column vector of any size, templated on scalar type.
template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

/// A matrix of 3 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

/// A matrix of 4 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix4X = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>;

/// A matrix of dynamic size, templated on scalar type.
template <typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

/// A column vector of dynamic size, up to a maximum of 73 elements.
using VectorUpTo73d = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73, 1>;

/// An autodiff variable with a dynamic number of partials, up to 73 maximum.
using AutoDiffUpTo73d = Eigen::AutoDiffScalar<VectorUpTo73d>;

/// An autodiff variable with a dynamic number of partials.
using AutoDiffXd = Eigen::AutoDiffScalar<Eigen::VectorXd>;

}  // namespace drake
