#include "drake/common/drake_deprecated.h"

#include <gtest/gtest.h>

// This test verifies that the Drake build still succeeds if a deprecated class
// or function is in use.

namespace {

class DRAKE_DEPRECATED("2038-01-19", "Use MyNewClass instead.") MyClass {
};

class MyNewClass {
};

DRAKE_DEPRECATED("2038-01-19",
    "Don't use this function; use NewMethod() instead.")
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
