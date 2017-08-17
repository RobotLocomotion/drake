#pragma once

#include <memory>

#include "drake/common/test/is_dynamic_castable.h"

namespace drake {
namespace systems {

/// Tests whether the given device under test of type S<double> can be
/// converted to use AutoDiffXd as its scalar type.  If the test passes,
/// additional checks on the converted object of type `const S<AutoDiffXd>&`
/// can be passed via a lambda into @p callback.  The Callback must take an
/// `const S<AutoDiffXd>&` and return void; a typical value would be a lambda
/// such as `[](const auto& converted) { EXPECT_TRUE(converted.thing()); }`.
template <template <typename> class S, typename Callback>
::testing::AssertionResult is_autodiffxd_convertible(
     const S<double>& dut, Callback callback) {
  // Check if a proper type came out; return early if not.
  std::unique_ptr<System<AutoDiffXd>> converted = dut.ToAutoDiffXd();
  ::testing::AssertionResult result =
        is_dynamic_castable<S<AutoDiffXd>>(converted);
  if (!result) { return result; }

  // Allow calling code to specify additional tests on the converted System.
  const S<AutoDiffXd>& downcast =
      dynamic_cast<const S<AutoDiffXd>&>(*converted);
  callback(downcast);

  return ::testing::AssertionSuccess();
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use AutoDiffXd as its scalar type.
template <template <typename> class S>
::testing::AssertionResult is_autodiffxd_convertible(const S<double>& dut) {
  return is_autodiffxd_convertible(dut, [](const auto&){});
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use Expression as its scalar type.  If the test passes,
/// additional checks on the converted object of type `const S<Expression>&`
/// can be passed via a lambda into @p callback.  The Callback must take an
/// `const S<Expression>&` and return void; a typical value would be a lambda
/// such as `[](const auto& converted) { EXPECT_TRUE(converted.thing()); }`.
template <template <typename> class S, typename Callback>
::testing::AssertionResult is_symbolic_convertible(
     const S<double>& dut, Callback callback) {
  // Check if a proper type came out; return early if not.
  std::unique_ptr<System<symbolic::Expression>> converted = dut.ToSymbolic();
  ::testing::AssertionResult result =
        is_dynamic_castable<S<symbolic::Expression>>(converted);
  if (!result) { return result; }

  // Allow calling code to specify additional tests on the converted System.
  const S<symbolic::Expression>& downcast =
      dynamic_cast<const S<symbolic::Expression>&>(*converted);
  callback(downcast);

  return ::testing::AssertionSuccess();
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use Expression as its scalar type.
template <template <typename> class S>
::testing::AssertionResult is_symbolic_convertible(const S<double>& dut) {
  return is_symbolic_convertible(dut, [](const auto&){});
}

}  // namespace systems
}  // namespace drake
