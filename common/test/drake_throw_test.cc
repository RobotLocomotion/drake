#include "drake/common/drake_throw.h"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace {

GTEST_TEST(DrakeThrowTest, BasicTest) {
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true));
  EXPECT_THROW(DRAKE_THROW_UNLESS(false), std::runtime_error);
}

// The test tests two things:
//
//   1. Regression guaranteeing that four value expressions are supported.
//   2. Provided value expressions get encoded as expected.
//
// If we have hiccups with value expressions of various types, these tests can
// be extended to explicitly test those types.
GTEST_TEST(DrakeThrowTest, ThrowWithValues) {
  // Create a collection of lvalues.
  const double a = 1.5;
  const double b = 2.5;
  const double c = 3.5;
  const double d = 4.5;
  // Ensure it works with up to four values with a true-valued condition.
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, a));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, a, b));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, a, b, c));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, a, b, c, d));

  // Note: Our DRAKE_EXPECT_THROWS_MESSAGE doesn't like the do {} while(0) body
  // of the DRAKE_THROW_UNLESS expression. To placate it, we place the
  // invocation in a lambda to make things happier.
  auto do_throw = [&](int count) {
    switch (count) {
      case 1:
        DRAKE_THROW_UNLESS(false, a);
        return;
      case 2:
        DRAKE_THROW_UNLESS(false, a, b);
        return;
      case 3:
        DRAKE_THROW_UNLESS(false, a, b, c);
        return;
      case 4:
        DRAKE_THROW_UNLESS(false, a, b, c, d);
        return;
    }
    DRAKE_UNREACHABLE();
  };

  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(1), ".*a = 1.5.");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(2), ".*a = 1.5, b = 2.5.");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(3), ".*a = 1.5, b = 2.5, c = 3.5.");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(4),
                              ".*a = 1.5, b = 2.5, c = 3.5, d = 4.5.");
}

// We explicitly format some built-in types a certain way, confirm they get it.
GTEST_TEST(DrakeThrowTest, BuiltInFormatting) {
  auto throw_error = [](const auto& value) {
    DRAKE_THROW_UNLESS(false, value);
  };

  // All floating-point values have an enforced .0 to maintain compatibility
  // across platforms.
  const double d = 1;
  DRAKE_EXPECT_THROWS_MESSAGE(throw_error(d), ".*1.0.*");
  const float f = 2;
  DRAKE_EXPECT_THROWS_MESSAGE(throw_error(f), ".*2.0.*");
}

}  // namespace
