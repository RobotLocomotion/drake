#pragma once

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

namespace drake {

/// Checks if @p ptr, a pointer to `FromType` class, can be safely converted to
/// a pointer to `ToType` class. Our motivation is to provide a good diagnostic
/// for what @p ptr _actually_ was when the test fails.
///
/// Here is an illustrative code snippet. We assume that `Prius` and `Camry` are
/// derived classes of `Car`.
///
/// @code
/// const Car* prius = new Prius{};
///
/// // The following assertion fails without diagnostic info.
/// EXPECT_TRUE(dynamic_cast<Camry>(ptr) != nullptr)
///
/// // The following assertion returns `::testing::AssertionFailure()` with
/// // diagnostic info attached.
/// EXPECT_TRUE(is_dynamic_castable<Camry>(prius));
/// // Output:
/// // Value of: is_dynamic_castable<Camry>(prius)
/// // Actual: false (is_dynamic_castable<Camry>(Car* ptr) failed
/// //                because ptr is of dynamic type Prius.)
/// // Expected: true
/// @endcode
template <typename ToType, typename FromType>
[[nodiscard]] ::testing::AssertionResult is_dynamic_castable(
    const FromType* ptr) {
  if (ptr == nullptr) {
    const std::string from_name{NiceTypeName::Get<FromType>()};
    const std::string to_name{NiceTypeName::Get<ToType>()};
    return ::testing::AssertionFailure()
        << "is_dynamic_castable<" << to_name << ">(" << from_name << "* ptr)"
        << " failed because ptr was already nullptr.";
  }
  if (dynamic_cast<const ToType* const>(ptr) == nullptr) {
    const std::string from_name{NiceTypeName::Get<FromType>()};
    const std::string to_name{NiceTypeName::Get<ToType>()};
    const std::string dynamic_name{NiceTypeName::Get(*ptr)};
    return ::testing::AssertionFailure()
        << "is_dynamic_castable<" << to_name << ">(" << from_name << "* ptr)"
        << " failed because ptr is of dynamic type " << dynamic_name << ".";
  }
  return ::testing::AssertionSuccess();
}

/// Checks if @p ptr, a shared pointer to `FromType` class, can be safely
/// converted to a shared pointer to `ToType` class. Our motivation is to
/// provide a good diagnostic for what @p ptr _actually_ was when the test
/// fails.
template <typename ToType, typename FromType>
[[nodiscard]] ::testing::AssertionResult is_dynamic_castable(
    std::shared_ptr<FromType> ptr) {
  return is_dynamic_castable<ToType, FromType>(ptr.get());
}

/// Checks if @p ptr, a shared pointer to `FromType` class, can be safely
/// converted to a shared pointer to `ToType` class. Our motivation is to
/// provide a good diagnostic for what @p ptr _actually_ was when the test
/// fails.
template <typename ToType, typename FromType>
[[nodiscard]] ::testing::AssertionResult is_dynamic_castable(
    const std::unique_ptr<FromType>& ptr) {
  return is_dynamic_castable<ToType, FromType>(ptr.get());
}

}  // namespace drake
