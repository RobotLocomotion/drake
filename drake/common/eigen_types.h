#pragma once

/// @file
/// This file contains abbreviated definitions for certain specializations of
/// Eigen::Matrix that are commonly used in Drake.
/// These convenient definitions are templated on the scalar type of the Eigen
/// object. While Drake uses `<T>` for scalar types across the entire code base
/// we decided in this file to use `<Scalar>` to be more consistent with the
/// usage of `<Scalar>` in Eigen's code base.
/// @see also eigen_autodiff_types.h

#include <Eigen/Dense>

#include "drake/common/constants.h"

namespace drake {

/// A column vector of size 1 (that is, a scalar), templated on scalar type.
template <typename Scalar>
using Vector1 = Eigen::Matrix<Scalar, 1, 1>;

/// A column vector of size 1 of doubles.
using Vector1d = Eigen::Matrix<double, 1, 1>;

/// A column vector of size 3, templated on scalar type.
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

/// A column vector of size 4, templated on scalar type.
template <typename Scalar>
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

/// A column vector of size 6.
template <typename Scalar>
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;

/// A column vector of any size, templated on scalar type.
template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

/// A matrix of 3 rows and 3 columns, templated on scalar type.
template <typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

/// A matrix of 6 rows and 6 columns, templated on scalar type.
template <typename Scalar>
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

/// A matrix of 3 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

/// A matrix of 4 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix4X = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>;

/// A matrix of 6 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

/// A matrix of dynamic size, templated on scalar type.
template <typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

/// A quaternion templated on scalar type.
template <typename Scalar>
using Quaternion = Eigen::Quaternion<Scalar>;

/// An Isometry templated on scalar type.
template <typename Scalar>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

/// A column vector of dynamic size, up to a maximum of 73 elements.
using VectorUpTo73d = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73, 1>;

/// A column vector consisting of one twist.
template <typename Scalar>
using TwistVector = Eigen::Matrix<Scalar, kTwistSize, 1>;

/// A matrix with one twist per column, and dynamically many columns.
template <typename Scalar>
using TwistMatrix = Eigen::Matrix<Scalar, kTwistSize, Eigen::Dynamic>;

/// A six-by-six matrix.
template <typename Scalar>
using SquareTwistMatrix = Eigen::Matrix<Scalar, kTwistSize, kTwistSize>;

/// A column vector consisting of one wrench.
/// This is the classical definition of a wrench: an encoding of a translational
/// force (`f`) applied at a positional vector (`r`).  The column vector is a
/// concatenation of a moment and the force: `[r X f, f]^T`.  Its definition
/// therefore depends on the point of application of the force ('r') and a
/// point around which torque is computed."
template <typename Scalar>
using WrenchVector = Eigen::Matrix<Scalar, 6, 1>;

/// A column vector consisting of a concatenated rotational and translational
/// force.  The wrench is a special case of a SpatialForce.  For a general
/// SpatialForce the rotational force can be a pure torque or the accumulation
/// of moments and need not necessarily be a function of the force term.
template <typename Scalar>
using SpatialForce = Eigen::Matrix<Scalar, 6, 1>;

}  // namespace drake
