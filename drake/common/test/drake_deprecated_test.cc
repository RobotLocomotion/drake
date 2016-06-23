#include "drake/common/drake_deprecated.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

/* This test only verifies that the Drake build can still succeed if a 
deprecated class or function is in use. I do not know how to test whether
the desired warnings are actually issued except by inspection. */

namespace {

class DRAKE_DEPRECATED("use MyNewClass instead") MyClass {
  int i{};
};

class MyNewClass {
  int j{};
};

DRAKE_DEPRECATED("don't use this function; use g() instead")
int f(int arg) { return arg; }

int g(int arg) { return arg; }

GTEST_TEST(DrakeDeprecatedTest, ClassTest) {
  volatile MyClass this_is_obsolete; // volatile to avoid "unused" warning
  volatile MyNewClass this_is_not;
}

GTEST_TEST(DrakeAssertDeathTest, FunctionTest) {
  volatile int obsolete = f(1);
  volatile int not_obsolete = g(1);
}

}  // namespace
