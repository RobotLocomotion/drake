#pragma once

/** @file
This header provides a single inclusion point for autodiff-related header files
in the `drake/common` directory. Users should include this file. Including other
individual headers such as `drake/common/autodiffxd.h` is not supported. */

#include "drake/common/autodiff_config.h"

#if DRAKE_INTERNAL_USE_EIGEN_LEGACY_AUTODIFF == 0

#include <Eigen/Core>

#include "drake/common/ad/auto_diff.h"

namespace drake {

/** A scalar type that performs automatic differentiation. Always use this
`AutoDiffXd` alias when referring to the scalar type. */
using AutoDiffXd = drake::ad::AutoDiff;

/** A dynamic-sized vector of autodiff variables. */
using AutoDiffVecXd = Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1>;

}  // namespace drake

#else  // DRAKE_INTERNAL_USE_EIGEN_LEGACY_AUTODIFF

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

namespace drake {

/** A scalar type that performs automatic differentiation. Always use this
`AutoDiffXd` alias when referring to the scalar type. */
using AutoDiffXd = Eigen::AutoDiffScalar<Eigen::VectorXd>;

/** A dynamic-sized vector of autodiff variables. */
using AutoDiffVecXd = Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1>;

}  // namespace drake

// Each Drake header below checks that this macro is defined and warns if not.
#define DRAKE_COMMON_AUTODIFF_HEADER

// Do not alpha-sort the following block of hard-coded #includes, which is
// protected by `clang-format` markers.
//
// Rationale: We want to maximize the use of this header, `autodiff.h`, even
// inside of the autodiff-related files to avoid any mistakes which might not be
// detected. By centralizing the list here, we make sure that everyone will see
// the correct order which respects the inter-dependencies of the autodiff
// headers. This shields us from triggering undefined behaviors due to
// order-of-specialization-includes-changed mistakes.
//
// clang-format off
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/autodiffxd.h"
#include "drake/common/autodiff_overloads.h"
// clang-format on
#undef DRAKE_COMMON_AUTODIFF_HEADER

#endif  // DRAKE_INTERNAL_USE_EIGEN_LEGACY_AUTODIFF
