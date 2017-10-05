#pragma once

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
// TODO(soonho-tri): Change to #error.
#warning Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <limits>
// If you see a 'duplicate specialization' compiler error here, then Eigen must
// have started providing this itself; conditionalize or remove Drake's own
// specialization.
namespace std {
template <typename T>
class numeric_limits<Eigen::AutoDiffScalar<T> >
  : public numeric_limits<typename T::Scalar> {};

}  // namespace std
