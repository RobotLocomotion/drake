#pragma once

/// @file
/// This file contains abbreviated definitions for certain uses of
/// AutoDiffScalar that are commonly used in Drake.
/// @see also eigen_types.h

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
#error Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <type_traits>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {

/// The Eigen vector type used by drake::AutoDiffXd for its partials.
using AutoDerXd = Eigen::Matrix<
  /* _Scalar = */ double,
  /* _Rows = */ Eigen::Dynamic,
  /* _Cols = */ 1,
  /* _Options = */ 0,
  /* _MaxRows = */ internal::kMaxRowsAtCompileTimeThatTriggersInlineStorage,
  /* _MaxCols = */ 1>;

/// An autodiff variable with a dynamic number of partials.
using AutoDiffXd = Eigen::AutoDiffScalar<AutoDerXd>;

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
