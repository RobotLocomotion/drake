#pragma once

#include <stdexcept>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// BasicVector is a semantics-free wrapper around an Eigen vector that
/// satisfies VectorInterface. Once constructed, its size is fixed.
/// @tparam T The type of the vector element.
template <typename T>
class BasicVector : public VectorInterface<T> {
 public:
  explicit BasicVector(int size) : values_(VectorX<T>::Zero(size)) {}

  ~BasicVector() override {}

  void set_value(const VectorX<T>& value) override {
    if (value.rows() != values_.rows()) {
      throw std::runtime_error(
          "Cannot set a BasicVector of size " + std::to_string(values_.rows()) +
          " with a value of size " + std::to_string(value.rows()));
    }
    values_ = value;
  }

  const VectorX<T>& get_value() const override { return values_; }

  Eigen::VectorBlock<VectorX<T>> get_mutable_value() override {
    return values_.head(values_.rows());
  }

 private:
  // The column vector of T values.
  VectorX<T> values_;
};

}  // namespace systems
}  // namespace drake
