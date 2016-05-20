#pragma once

#include <memory>
#include <stdexcept>

#include "drake/systems/framework/state_vector_interface.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// SystemStateVector is a concrete class template that implements
/// StateVectorInterface in a convenient manner for Systems that have no
/// component Systems, by owning and wrapping a VectorInterface<T>.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class SystemStateVector : public StateVectorInterface<T> {
 public:
  explicit SystemStateVector(std::unique_ptr<VectorInterface<T>> vector)
      : vector_(std::move(vector)) {}

  size_t size() const override { return vector_->get_value().rows(); }

  const T GetAtIndex(size_t index) const override {
    if (index >= size()) {
      throw std::runtime_error("Index " + std::to_string(index) +
                               "out of bounds for state vector of size " +
                               std::to_string(size()));
    }
    return vector_->get_value()[index];
  }

  void SetAtIndex(size_t index, const T& value) override {
    if (index >= size()) {
      throw std::runtime_error("Index " + std::to_string(index) +
                               "out of bounds for state vector of size " +
                               std::to_string(size()));
    }
    vector_->get_mutable_value()[index] = value;
  }

  void SetFromVector(const Eigen::VectorBlock<VectorX<T>>& value) override {
    vector_->set_value(value);
  }

 private:
  // SystemStateVector objects are neither copyable nor moveable.
  SystemStateVector(const SystemStateVector& other) = delete;
  SystemStateVector& operator=(const SystemStateVector& other) = delete;
  SystemStateVector(SystemStateVector&& other) = delete;
  SystemStateVector& operator=(SystemStateVector&& other) = delete;

  std::unique_ptr<VectorInterface<T>> vector_;
};

}  // namespace systems
}  // namespace drake
