#pragma once

/// @file
/// This file contains abbreviated definitions for certain uses of
/// AutoDiffScalar that are commonly used in Drake.
/// @see also eigen_types.h

#include <type_traits>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/eigen_autodiff_limits.h"
#include "drake/common/eigen_types.h"

namespace drake {

/// An autodiff variable with a dynamic number of partials, up to 73 maximum.
using AutoDiffUpTo73d = Eigen::AutoDiffScalar<VectorUpTo73d>;

/// An autodiff variable with a dynamic number of partials.
using AutoDiffXd = Eigen::AutoDiffScalar<Eigen::VectorXd>;

// TODO(hongkai-dai): Recursive template to get arbitrary gradient order.

/// An autodiff variable with `num_vars` partials.
template <int num_vars>
using AutoDiffd = Eigen::AutoDiffScalar<Eigen::Matrix<double, num_vars, 1> >;

/// A vector of `rows` autodiff variables, each with `num_vars` partials.
template <int num_vars, int rows>
using AutoDiffVecd = Eigen::Matrix<AutoDiffd<num_vars>, rows, 1>;

/// A dynamic-sized vector of autodiff variables, each with a dynamic-sized
/// vector of partials.
typedef AutoDiffVecd<Eigen::Dynamic, Eigen::Dynamic> AutoDiffVecXd;

}  // namespace drake
