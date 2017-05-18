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

 private:
  void CheckInvariants() {
    DRAKE_DEMAND(owned_value_.get() == this->get_value());
  }

  std::unique_ptr<BasicVector<T>> owned_value_;
};

}  // namespace systems
}  // namespace drake
