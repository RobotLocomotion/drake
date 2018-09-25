#pragma once

// Redundant #define guards for the benefit of @pybind11//:mkdoc.py.
#ifndef DRAKE_COMMON_EIGEN_AUTODIFF_LIMITS_H_
#define DRAKE_COMMON_EIGEN_AUTODIFF_LIMITS_H_

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
// TODO(soonho-tri): Change to #error.
#warning Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <limits>

// Eigen provides `numeric_limits<AutoDiffScalar<T>>` starting with v3.3.4.
#if !EIGEN_VERSION_AT_LEAST(3, 3, 4)  // Eigen Version < v3.3.4

namespace std {
template <typename T>
class numeric_limits<Eigen::AutoDiffScalar<T>>
    : public numeric_limits<typename T::Scalar> {};

}  // namespace std

#endif  // Eigen Version < v3.3.4

#endif  // DRAKE_COMMON_EIGEN_AUTODIFF_LIMITS_H_
