#pragma once

/** @file
This header provides a single inclusion point for autodiff-related header files
in the `drake/common` directory. Users should include this file. Including other
individual headers is not supported. */

#include <Eigen/Core>

#include "drake/common/ad/auto_diff.h"

namespace drake {

/** A scalar type that performs automatic differentiation. Always use this
`AutoDiffXd` alias when referring to the scalar type. */
using AutoDiffXd = drake::ad::AutoDiff;

/** A dynamic-sized vector of autodiff variables. */
using AutoDiffVecXd = Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1>;

}  // namespace drake
