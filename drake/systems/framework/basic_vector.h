#pragma once

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// BasicVector is a semantics-free wrapper around an Eigen vector that
/// satisfies VectorInterface. Once constructed, its size is fixed.
/// The BasicVector is initialized to the quiet_NaN of the Eigen scalar.
/// If numeric_limits is not specialized on the Eigen scalar, the BasicVector
/// will be initialized with the scalar's default constructor.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class BasicVector : public VectorInterface<T> {
 public:
  explicit BasicVector(int size)
      : values_(VectorX<T>::Constant(
            size, std::numeric_limits<
                      typename Eigen::NumTraits<T>::Real>::quiet_NaN())) {}

  ~BasicVector() override {}

  void set_value(const Eigen::Ref<const VectorX<T>>& value) override {
    if (value.rows() != values_.rows()) {
      throw std::out_of_range(
          "Cannot set a BasicVector of size " + std::to_string(values_.rows()) +
          " with a value of size " + std::to_string(value.rows()));
    }
    values_ = value;
  }

  int size() const override { return values_.rows(); }

  Eigen::VectorBlock<const VectorX<T>> get_value() const override {
    return values_.head(values_.rows());
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_value() override {
    return values_.head(values_.rows());
  }

  /// Copies the entire state to a new BasicVector.
  ///
  /// Uses the Non-Virtual Interface idiom because smart pointers do not have
  /// type covariance.
  std::unique_ptr<VectorInterface<T>> Clone() const final {
    return std::unique_ptr<VectorInterface<T>>(DoClone());
  }

 private:
  // Returns a new BasicStateVector containing a copy of the entire state.
  // Subclasses of BasicVector must override DoClone to return their covariant
  // type.
  virtual BasicVector<T>* DoClone() const {
    BasicVector* clone = new BasicVector(size());
    clone->values_ = values_;
    return clone;
  }

  // The column vector of T values.
  VectorX<T> values_;
};

}  // namespace systems
}  // namespace drake
