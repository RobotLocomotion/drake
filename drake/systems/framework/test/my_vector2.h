#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace test {

/// A simple subclass of BasicVector<T> for testing, particularly for cases
/// where BasicVector subtyping must be preserved through the framework.
template <typename T>
class MyVector2 : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyVector2)
  MyVector2() : BasicVector<T>(2) {}

  MyVector2* DoClone() const override {
    auto result = new MyVector2;
    result->set_value(this->get_value());
    return result;
  }

  /// Creates a new MyVector2 with value [a, b].
  static std::unique_ptr<MyVector2> Make(const T& a, const T& b) {
    auto result = std::make_unique<MyVector2>();
    result->SetAtIndex(0, a);
    result->SetAtIndex(1, b);
    return result;
  }
};

}  // namespace test
}  // namespace systems
}  // namespace drake
