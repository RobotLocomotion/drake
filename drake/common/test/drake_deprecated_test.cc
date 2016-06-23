#include "drake/common/drake_deprecated.h"

#include "gtest/gtest.h"

/* This test only verifies that the Drake build can still succeed if a
deprecated class or function is in use.

Note: It would be possible to test whether warnings are actually issued by
building this test with compiler flags making the deprecated warning an error
(-Werror=deprecated-declarations for gcc and clang), then using the technique
used for drake_assert_test_compile to check for failure to compile. We are not
doing that here. */

namespace {

class DRAKE_DEPRECATED("Use MyNewClass instead.") MyClass {
};

class MyNewClass {
};

DRAKE_DEPRECATED("Don't use this function; use NewMethod() instead.")
int OldMethod(int arg) { return arg; }

int NewMethod(int arg) { return arg; }

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

}  // namespace
