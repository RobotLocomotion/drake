#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// BasicStateVector is a concrete class template that implements
/// StateVector in a convenient manner for LeafSystem blocks,
/// by owning and wrapping a VectorBase<T>.
///
/// It will often be convenient to inherit from BasicStateVector, and add
/// additional semantics specific to the LeafSystem. Such child classes must
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
            std::unique_ptr<VectorBase<T>>(new BasicVector<T>(size))) {}

  /// Constructs a BasicStateVector that owns a generic BasicVector with the
  /// specified @p data.
  explicit BasicStateVector(const std::vector<T>& data)
      : BasicStateVector(static_cast<int>(data.size())) {
    for (int i = 0; i < static_cast<int>(data.size()); ++i) {
      SetAtIndex(i, data[i]);
    }
  }

  /// Constructs a BasicStateVector that owns an arbitrary @p vector, which
  /// must not be nullptr.
  explicit BasicStateVector(std::unique_ptr<VectorBase<T>> vector)
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
  // Clone other's wrapped vector, in case is it not a BasicVector.
  BasicStateVector(const BasicStateVector& other)
      : BasicStateVector(other.vector_->CloneVector()) {}

  BasicStateVector<T>* DoClone() const override {
    return new BasicStateVector<T>(*this);
  }

  /// Returns a mutable reference to the underlying VectorBase.
  VectorBase<T>& get_wrapped_vector() { return *vector_; }
  /// Returns a const reference to the underlying VectorBase.
  const VectorBase<T>& get_wrapped_vector() const { return *vector_; }

 private:
  // Assignment of BasicStateVectors could change size, so we forbid it.
  BasicStateVector& operator=(const BasicStateVector& other) = delete;

  // BasicStateVector objects are not moveable.
  BasicStateVector(BasicStateVector&& other) = delete;
  BasicStateVector& operator=(BasicStateVector&& other) = delete;

  std::unique_ptr<VectorBase<T>> vector_;
};

}  // namespace systems
}  // namespace drake
