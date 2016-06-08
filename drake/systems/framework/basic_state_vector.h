#pragma once

#include <memory>
#include <stdexcept>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector_interface.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// BasicStateVector is a concrete class template that implements
/// StateVectorInterface in a convenient manner for leaf Systems,
/// by owning and wrapping a VectorInterface<T>.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class BasicStateVector : public StateVectorInterface<T> {
 public:
  /// Constructs a BasicStateVector that owns a generic BasicVector of the
  /// specified @p size.
  explicit BasicStateVector(int64_t size)
      : BasicStateVector(
            std::unique_ptr<VectorInterface<T>>(new BasicVector<T>(size))) {}

  /// Constructs a BasicStateVector that owns an arbitrary @p vector.
  explicit BasicStateVector(std::unique_ptr<VectorInterface<T>> vector)
      : vector_(std::move(vector)) {}

  ptrdiff_t size() const override { return vector_->get_value().rows(); }

  const T GetAtIndex(ptrdiff_t index) const override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              "out of bounds for state vector of size " +
                              std::to_string(size()));
    }
    return vector_->get_value()[index];
  }

  void SetAtIndex(ptrdiff_t index, const T& value) override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              "out of bounds for state vector of size " +
                              std::to_string(size()));
    }
    vector_->get_mutable_value()[index] = value;
  }

  void SetFromVector(const Eigen::Ref<VectorX<T>>& value) override {
    vector_->set_value(value);
  }

 private:
  // BasicStateVector objects are neither copyable nor moveable.
  BasicStateVector(const BasicStateVector& other) = delete;
  BasicStateVector& operator=(const BasicStateVector& other) = delete;
  BasicStateVector(BasicStateVector&& other) = delete;
  BasicStateVector& operator=(BasicStateVector&& other) = delete;

  std::unique_ptr<VectorInterface<T>> vector_;
};

}  // namespace systems
}  // namespace drake
