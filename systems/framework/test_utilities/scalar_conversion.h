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

namespace internal {
// This is the implementation for all of the public conversion check functions
// below. It evaluates both static and dynamic limitations on conversion, and
// returns AssertionSuccess() if conversion is supported. Otherwise, it returns
// AssertionFailure() with a detailed error message.
//
// @param dut the system to check for conversion support.
// @param callback receives the converted system, for additional checking; see
//                 public documentation below.
// @tparam ToScalar the proposed destination scalar type.
// @tparam S the type of the system to convert.
// @tparam Callback  the type of the callback.
template <typename ToScalar, template <typename> class S, typename Callback>
::testing::AssertionResult is_convertible_to(const S<double>& dut,
                                             Callback callback) {
  using Traits = typename scalar_conversion::Traits<S>;
  if constexpr (Traits::template supported<ToScalar, double>::value) {
    // Check if a proper type came out; return early if not.
    std::unique_ptr<System<ToScalar>> converted =
        dut.template ToScalarTypeMaybe<ToScalar>();
    ::testing::AssertionResult result =
        is_dynamic_castable<S<ToScalar>>(converted);
    if (!result) {
      return result;
    }

    // Allow calling code to specify additional tests on the converted System.
    const S<ToScalar>& downcast = dynamic_cast<const S<ToScalar>&>(*converted);
    callback(downcast);

    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure() << fmt::format(
               "scalar conversion from 'double' to '{}' statically forbidden "
               "for '{}' by traits '{}'",
               NiceTypeName::Get<ToScalar>(), NiceTypeName::Get<S<double>>(),
               NiceTypeName::Get<Traits>());
  }
}
}  // namespace internal

/// Tests whether the given device under test of type S<double> can be
/// converted to use AutoDiffXd as its scalar type.  If the test passes,
/// additional checks on the converted object of type `const S<AutoDiffXd>&`
/// can be passed via a lambda into @p callback.  The Callback must take an
/// `const S<AutoDiffXd>&` and return void; a typical value would be a lambda
/// such as `[](const auto& converted) { EXPECT_TRUE(converted.thing()); }`.
template <template <typename> class S, typename Callback>
::testing::AssertionResult is_autodiffxd_convertible(const S<double>& dut,
                                                     Callback callback) {
  return internal::is_convertible_to<AutoDiffXd>(dut, callback);
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use AutoDiffXd as its scalar type.
template <template <typename> class S>
::testing::AssertionResult is_autodiffxd_convertible(const S<double>& dut) {
  return is_autodiffxd_convertible(dut, [](const auto&) {});
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use Expression as its scalar type.  If the test passes,
/// additional checks on the converted object of type `const S<Expression>&`
/// can be passed via a lambda into @p callback.  The Callback must take an
/// `const S<Expression>&` and return void; a typical value would be a lambda
/// such as `[](const auto& converted) { EXPECT_TRUE(converted.thing()); }`.
template <template <typename> class S, typename Callback>
::testing::AssertionResult is_symbolic_convertible(const S<double>& dut,
                                                   Callback callback) {
  return internal::is_convertible_to<symbolic::Expression>(dut, callback);
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use Expression as its scalar type.
template <template <typename> class S>
::testing::AssertionResult is_symbolic_convertible(const S<double>& dut) {
  return is_symbolic_convertible(dut, [](const auto&) {});
}

}  // namespace systems
}  // namespace drake
