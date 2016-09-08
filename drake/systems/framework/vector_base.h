#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/state_vector.h"

namespace drake {
namespace systems {

/// VectorBase is a pure abstract interface that real-valued signals
/// between Systems and real-valued System state vectors must satisfy.
/// Classes that inherit from VectorBase will typically provide names
/// for the elements of the vector, and may also provide other
/// computations for the convenience of Systems handling the
/// signal. The vector is always a column vector. It may or may not
/// be contiguous in memory. Contiguous subclasses should typically
/// inherit from BasicVector, not from VectorBase directly.
///
/// @tparam T Must be a Scalar compatible with Eigen.
template <typename T>
class VectorBase : public StateVector<T> {
 public:
  virtual ~VectorBase() {}

  // VectorBase objects are neither copyable nor moveable.
  VectorBase(const VectorBase<T>& other) = delete;
  VectorBase& operator=(const VectorBase<T>& other) = delete;
  VectorBase(VectorBase<T>&& other) = delete;
  VectorBase& operator=(VectorBase<T>&& other) = delete;

 protected:
  VectorBase() {}
};

}  // namespace systems
}  // namespace drake
