#pragma once

#include <Eigen/Core>

#include "drake/common/autodiff/auto_diff.h"

namespace drake {

/// An autodiff variable with a dynamic number of partials.
using AutoDiffXd = autodiff::AutoDiff;

/// A dynamic-sized vector of autodiff variables, each with a dynamic-sized
/// vector of partials.
using AutoDiffVecXd = Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1>;

}  // namespace drake

#define DRAKE_COMMON_AUTODIFF_HEADER
#include "drake/common/autodiff_overloads.h"
#undef DRAKE_COMMON_AUTODIFF_HEADER
