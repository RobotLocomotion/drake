#pragma once

#include <stdexcept>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// BasicVector is a semantics-free wrapper around an Eigen vector that
/// satisfies VectorInterface.
template <typename ScalarType>
class BasicVector : public VectorInterface<ScalarType> {
 public:
  explicit BasicVector(int size)
      : values_(VectorX<ScalarType>::Zero(size, 1 /* column */)) {}

  ~BasicVector() override {}

  void set_value(const VectorX<ScalarType>& value) override {
    if (value.rows() != values_.rows()) {
      throw std::runtime_error(
          "Cannot set a BasicVector of size " + std::to_string(values_.rows()) +
          " with a value of size " + std::to_string(value.rows()));
    }
    values_ = value;
  }

  const VectorX<ScalarType>& get_value() const override { return values_; }

  Eigen::VectorBlock<VectorX<ScalarType>> get_mutable_value() override {
    return values_.head(values_.rows());
  }

 private:
  // The column vector of ScalarType values.
  VectorX<ScalarType> values_;
};

}  // namespace systems
}  // namespace drake
