#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// A container class for BasicVector<T>.
///
/// @tparam T The type of the vector data. Must be a valid Eigen scalar.
template <typename T>
class VectorValue : public Value<BasicVector<T>*> {
 public:
  explicit VectorValue(std::unique_ptr<BasicVector<T>> v)
      : Value<BasicVector<T>*>(v.get()), owned_value_(std::move(v)) {
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  ~VectorValue() override {}

  // VectorValues are copyable but not moveable.
  explicit VectorValue(const VectorValue& other)
      : Value<BasicVector<T>*>(nullptr) {
    if (other.get_value() != nullptr) {
      owned_value_ = other.get_value()->Clone();
      this->set_value(owned_value_.get());
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  VectorValue& operator=(const VectorValue& other) {
    if (this == &other) {
      // Special case to do nothing, to avoid an unnecessary Clone.
    } else if (other.get_value() == nullptr) {
      owned_value_.reset();
      this->set_value(owned_value_.get());
    } else {
      owned_value_ = other.get_value()->Clone();
      this->set_value(owned_value_.get());
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::make_unique<VectorValue>(*this);
  }

  /// Obtain a const reference to the BasicVector owned by this VectorValue.
  const BasicVector<T>& get_vector() const {
    DRAKE_ASSERT(owned_value_ != nullptr);
    return *owned_value_;
  }

  /// Obtain a mutable reference to the BasicVector owned by this VectorValue.
  BasicVector<T>& get_mutable_vector() {
    DRAKE_ASSERT(owned_value_ != nullptr);
    return *owned_value_;
  }

  /// Extract the contained BasicVector and transfer ownership to
  /// the caller. The VectorValue is left empty. This is useful when you have
  /// been handed an AbstractValue but would like to use only the BasicVector
  /// you know lurks inside. After extracting the vector you should delete
  /// the now-empty shell of the AbstractValue.
  std::unique_ptr<BasicVector<T>> release_vector() {
    this->set_value(nullptr);  // Clear the parent Value.
    return std::move(owned_value_);
  }

 private:
  void CheckInvariants() {
    DRAKE_DEMAND(owned_value_.get() == this->get_value());
  }

  std::unique_ptr<BasicVector<T>> owned_value_;
};

}  // namespace systems
}  // namespace drake
