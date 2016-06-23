#include "drake/common/drake_deprecated.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

/* This test only verifies that the Drake build can still succeed if a 
deprecated class or function is in use. I do not know how to test whether
the desired warnings are actually issued except by inspection. */

namespace {

class DRAKE_DEPRECATED("use MyNewClass instead") MyClass {
};

class MyNewClass {
};

DRAKE_DEPRECATED("don't use this function; use g() instead")
int f(int arg) { return arg; }

int g(int arg) { return arg; }

GTEST_TEST(DrakeDeprecatedTest, ClassTest) {
  MyClass this_is_obsolete;
  MyNewClass this_is_not;
  (void)this_is_obsolete; // avoid "unused" warning
  (void)this_is_not;
}

GTEST_TEST(DrakeAssertDeathTest, FunctionTest) {
  int obsolete = f(1);
  int not_obsolete = g(1);
  (void)obsolete;
  (void)not_obsolete;
}

}  // namespace
