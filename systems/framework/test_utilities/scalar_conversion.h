#pragma once

#include <memory>

#include "drake/common/test_utilities/is_dynamic_castable.h"

namespace drake {
namespace systems {

namespace test {
// A helper function incompatible with symbolic::Expression.  This is useful to
// prove that scalar conversion code does not even instantiate this function
// when T = symbolic::Expression when told not to.  If it did, we see compile
// errors that this function does not compile with T = symbolic::Expression.
template <typename T>
static T copysign_int_to_non_symbolic_scalar(int magic, const T& value) {
  if ((magic < 0) == (value < 0.0)) {
    return value;
  } else {
    return -value;
  }
}
}  // namespace test

/// Tests whether the given device under test of type S<double> can be
/// converted to use AutoDiffXd as its scalar type.  If the test passes,
/// additional checks on the converted object of type `const S<AutoDiffXd>&`
/// can be passed via a lambda into @p callback.  The Callback must take an
/// `const S<AutoDiffXd>&` and return void; a typical value would be a lambda
/// such as `[](const auto& converted) { EXPECT_TRUE(converted.thing()); }`.
template <template <typename> class S, typename Callback>
::testing::AssertionResult is_autodiffxd_convertible(
     const S<double>& dut, Callback callback) {
  // We must use salted local variable names ("_67273" suffix) to work around
  // GCC 5.4 bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67273 because the
  // `callback` is a generic lambda.  The bug is fixed as of GCC 6.1.

  // Check if a proper type came out; return early if not.
  std::unique_ptr<System<AutoDiffXd>> converted_67273 =
      dut.ToAutoDiffXdMaybe();
  ::testing::AssertionResult result_67273 =
        is_dynamic_castable<S<AutoDiffXd>>(converted_67273);
  if (!result_67273) { return result_67273; }

  // Allow calling code to specify additional tests on the converted System.
  const S<AutoDiffXd>& downcast_67273 =
      dynamic_cast<const S<AutoDiffXd>&>(*converted_67273);
  callback(downcast_67273);

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
  // We must use salted local variable names ("_67273" suffix) to work around
  // GCC 5.4 bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67273 because the
  // `callback` is a generic lambda.  The bug is fixed as of GCC 6.1.

  // Check if a proper type came out; return early if not.
  std::unique_ptr<System<symbolic::Expression>> converted_67273 =
      dut.ToSymbolicMaybe();
  ::testing::AssertionResult result_67273 =
        is_dynamic_castable<S<symbolic::Expression>>(converted_67273);
  if (!result_67273) { return result_67273; }

  // Allow calling code to specify additional tests on the converted System.
  const S<symbolic::Expression>& downcast_67273 =
      dynamic_cast<const S<symbolic::Expression>&>(*converted_67273);
  callback(downcast_67273);

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
