#include "drake/common/drake_deprecated.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

// This test verifies that the Drake build still succeeds if a deprecated class
// or function is in use.

namespace {

class DRAKE_DEPRECATED("2038-01-19", "Use MyNewClass instead.") MyClass {};

class MyNewClass {};

DRAKE_DEPRECATED("2038-01-19",
                 "Don't use this function; use NewMethod() instead.")
int OldMethod(int arg) {
  return arg;
}

int NewMethod(int arg) {
  return arg;
}

GTEST_TEST(DrakeDeprecatedTest, ClassTest) {
  MyClass this_is_obsolete;
  MyNewClass this_is_not;
  (void)this_is_obsolete;  // Avoid "unused" warning.
  (void)this_is_not;
}

GTEST_TEST(DrakeDeprecatedTest, FunctionTest) {
  int obsolete = OldMethod(1);
  int not_obsolete = NewMethod(1);
  (void)obsolete;
  (void)not_obsolete;
}

// Check that the "warn once" idiom compiles and doesn't crash at runtime.
GTEST_TEST(DrakeDeprecatedTest, WarnOnceTest) {
  static const drake::internal::WarnDeprecated warn_once(
      "2038-01-19", "The method OldCalc() has been renamed to NewCalc().");
}

// When the magic environment variable is set, warnings become errors.
GTEST_TEST(DrakeDeprecatedTest, WarnThrowsTest) {
  constexpr char kEnvName[] = "_DRAKE_DEPRECATION_IS_ERROR";
  ASSERT_EQ(::setenv(kEnvName, "1", 1), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      drake::internal::WarnDeprecated("2038-01-19", "Hello"),
      "DRAKE DEPRECATED: Hello. The deprecated code will be removed from Drake "
      "on or after 2038-01-19.");
  ASSERT_EQ(::unsetenv(kEnvName), 0);
}

}  // namespace
