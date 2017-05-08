#pragma once

#include <limits>

namespace std {
template <typename T>
class numeric_limits<Eigen::AutoDiffScalar<T> >
  : public numeric_limits<typename T::Scalar> {};

}  // namespace std
