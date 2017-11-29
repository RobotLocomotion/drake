#pragma once

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
