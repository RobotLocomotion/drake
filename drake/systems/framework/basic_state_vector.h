#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/vector_interface.h"
#include "leaf_state_vector.h"

namespace drake {
namespace systems {

/// BasicStateVector is a concrete class template that implements
/// StateVector in a convenient manner for leaf Systems,
/// by owning and wrapping a VectorInterface<T>.
///
/// It will often be convenient to inherit from BasicStateVector, and add
/// additional semantics specific to the leaf System. Such child classes must
/// override DoClone with an implementation that returns their concrete type.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class BasicStateVector : public LeafStateVector<T> {
 public:
  /// Constructs a BasicStateVector that owns a generic BasicVector of the
  /// specified @p size.
  explicit BasicStateVector(int size)
      : BasicStateVector(
            std::unique_ptr<VectorInterface<T>>(new BasicVector<T>(size))) {}

  /// Constructs a BasicStateVector that owns a generic BasicVector with the
  /// specified @p data.
  explicit BasicStateVector(const std::vector<T>& data)
      : BasicStateVector(data.size()) {
    for (size_t i = 0; i < data.size(); ++i) {
      SetAtIndex(i, data[i]);
    }
  }

  /// Constructs a BasicStateVector that owns an arbitrary @p vector, which
  /// must not be nullptr.
  explicit BasicStateVector(std::unique_ptr<VectorInterface<T>> vector)
      : vector_(std::move(vector)) {}

  int size() const override {
    return static_cast<int>(vector_->get_value().rows());
  }

  const T GetAtIndex(int index) const override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for state vector of size " +
                              std::to_string(size()));
    }
    return vector_->get_value()[index];
  }

  void SetAtIndex(int index, const T& value) override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for state vector of size " +
                              std::to_string(size()));
    }
    vector_->get_mutable_value()[index] = value;
  }

  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) override {
    vector_->set_value(value);
  }

  VectorX<T> CopyToVector() const override { return vector_->get_value(); }

  void ScaleAndAddToVector(const T& scale,
                           Eigen::Ref<VectorX<T>> vec) const override {
    if (vec.rows() != size()) {
      throw std::out_of_range("Addends must be the same length.");
    }
    vec += scale * vector_->get_value();
  }

  BasicStateVector& PlusEqScaled(const T& scale,
                                 const StateVector<T>& rhs) override {
    rhs.ScaleAndAddToVector(scale, vector_->get_mutable_value());
    return *this;
  }

 protected:
  BasicStateVector(const BasicStateVector& other)
      : BasicStateVector(other.size()) {
    SetFromVector(other.vector_->get_value());
  }

 private:
  BasicStateVector<T>* DoClone() const override {
    return new BasicStateVector<T>(*this);
  }

  // Assignment of BasicStateVectors could change size, so we forbid it.
  BasicStateVector& operator=(const BasicStateVector& other) = delete;

  // BasicStateVector objects are not moveable.
  BasicStateVector(BasicStateVector&& other) = delete;
  BasicStateVector& operator=(BasicStateVector&& other) = delete;

  std::unique_ptr<VectorInterface<T>> vector_;
};

}  // namespace systems
}  // namespace drake
