#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorValue)

  explicit VectorValue(std::unique_ptr<BasicVector<T>> v)
      : Value<BasicVector<T>*>(v.get()), owned_value_(std::move(v)) {
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  ~VectorValue() override {}

  std::unique_ptr<AbstractValue> Clone() const override {
    std::unique_ptr<BasicVector<T>> copy_of_owned;
    if (owned_value_) {
      copy_of_owned = owned_value_->Clone();
    }
    return std::make_unique<VectorValue>(std::move(copy_of_owned));
  }

 private:
  void CheckInvariants() {
    DRAKE_DEMAND(owned_value_.get() == this->get_value());
  }

  std::unique_ptr<BasicVector<T>> owned_value_;
};

}  // namespace systems
}  // namespace drake
