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
  const int b = 2;
  const double c = 3.5;
  const std::size_t d = 4;
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
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(2), ".*a = 1.5, b = 2.");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(3), ".*a = 1.5, b = 2, c = 3.5.");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(4), ".*a = 1.5, b = 2, c = 3.5, d = 4.");
}

// Confirms that the overload defined in fmt_eigen.h plays properly with the
// macro. Specifically, that the logic for stripping out `fmt_eigen(` from the
// value expression name works and that it otherwise compiles. We don't bother
// testing all Eigen types supported by fmt_eigen; we assume that has been
// tested elsewhere.
GTEST_TEST(DrakeThrowTest, ThrowWithFmtEigen) {
  const Eigen::Vector3d vec(1.5, 2.5, 3.5);
  const Eigen::Matrix2d mat = Eigen::Matrix2d::Identity();

  using drake::fmt_eigen;

  // Ensure that we can pass in a fmt_eigen_ref value expression.
  auto do_throw = [&](int case_number) {
    switch (case_number) {
      case 1:
        DRAKE_THROW_UNLESS(false, fmt_eigen(vec));
        return;
      case 2:
        DRAKE_THROW_UNLESS(false, fmt_eigen(vec.transpose()));
        return;
      case 3:
        // We strip out "drake::fmt_eigen" just like plain, old "fmt_eigen".
        DRAKE_THROW_UNLESS(false, drake::fmt_eigen(vec.transpose()));
        return;
      case 4:
        DRAKE_THROW_UNLESS(false, fmt_eigen(mat));
        return;
    }
    DRAKE_UNREACHABLE();
  };

  // Note: we're trying to ignore *how* the vector gets formatted and just
  // confirming that there's a sequence of numbers starting with 1 and ending
  // with 3.5.
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(1), ".*vec = .*1[^]*3.5.*");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(2),
                              ".*vec.transpose\\(\\) = .*1[^]*3.5.*");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(3),
                              ".*vec.transpose\\(\\) = .*1[^]*3.5.*");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(4), ".*mat = .*1[^]*1.*");
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

// Confirm that printable, string-like characters are accepted and get formatted
// with single quotes.
GTEST_TEST(DrakeThrowTest, ThrowWithFmtString) {
  const std::string str = "lvalue string";
  const std::string empty;
  const char* c_str = "c-string literal";
  const std::string_view sv = "string view";

  auto do_throw = [&](int case_number) {
    switch (case_number) {
      case 1:
        DRAKE_THROW_UNLESS(false, str);
        return;
      case 2:
        DRAKE_THROW_UNLESS(false, c_str);
        return;
      case 3:
        DRAKE_THROW_UNLESS(false, sv);
        return;
      case 4:
        DRAKE_THROW_UNLESS(false, empty);
        return;
    }
    DRAKE_UNREACHABLE();
  };

  // Note: we're confirming that the string values get decorated with double
  // quotes.
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(1), ".*str = \"lvalue string\".*");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(2), ".*c_str = \"c-string literal\".*");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(3), ".*sv = \"string view\".*");
  DRAKE_EXPECT_THROWS_MESSAGE(do_throw(4), ".*empty = \"\".*");
}

// Confirm behavior when the value expressions contain heterogeneous types.
GTEST_TEST(DrakeThrowTest, ThrowWithHeterogeneousValues) {
  const double d = 3.14;
  const std::string str = "pi";
  const float f = 2.71f;
  const char* c_str = "e";

  auto do_throw = [&]() {
    DRAKE_THROW_UNLESS(false, d, f, str, c_str);
  };

  DRAKE_EXPECT_THROWS_MESSAGE(
      do_throw(), ".*d = 3.14, f = 2.71, str = \"pi\", c_str = \"e\".*");
}

}  // namespace
