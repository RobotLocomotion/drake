#include "drake/common/drake_throw.h"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/fmt_ostream.h"
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
  const std::string msg = "a string";
  const double d = 7.5;
  const char c = 'x';
  const char bytes[] = "beef";
  // const Eigen::Vector3d bytes(1, 2, 3);

  // Ensure it works with up to four values.
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d, c));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d, c, msg));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d, c, msg, bytes));

  // Note: Our DRAKE_EXPECT_THROWS_MESSAGE doesn't like the do {} while(0) body
  // of the DRAKE_THROW_UNLESS expression. To placate it, we place the
  // invocation in a lambda to make things happier.
  auto f1 = [&](int count) {
    switch (count) {
      case 1:
        DRAKE_THROW_UNLESS(false, d);
        return;
      case 2:
        DRAKE_THROW_UNLESS(false, d, msg);
        return;
      case 3:
        DRAKE_THROW_UNLESS(false, d, msg, c);
        return;
      case 4:
        DRAKE_THROW_UNLESS(false, d, msg, c, bytes);
        return;
    }
    DRAKE_UNREACHABLE();
  };

  DRAKE_EXPECT_THROWS_MESSAGE(f1(1), ".*d = 7.5.*");
  DRAKE_EXPECT_THROWS_MESSAGE(f1(2), ".*d = 7.5, msg = \"a string\".");
  DRAKE_EXPECT_THROWS_MESSAGE(f1(3), ".*d = 7.5, msg = \"a string\", c = 'x'.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      f1(4), ".*d = 7.5, msg = \"a string\", c = 'x', bytes = \"beef\".");
}

GTEST_TEST(DrakeThrowTest, ThrowWithEigenValues) {
  const std::string details = "These are details";
  const Eigen::Vector3d v1(1, 2, 3);

  auto f1 = [&]() {
    DRAKE_THROW_UNLESS(false, drake::fmt_eigen(v1), details);
  };
  DRAKE_EXPECT_THROWS_MESSAGE(f1(), ".* v1 = 1\n2\n3, details = .*");

  // TODO: Why does this work without fmt_eigen?
  auto f2 = [&]() {
    DRAKE_THROW_UNLESS(false, v1.transpose(), details);
  };
  DRAKE_EXPECT_THROWS_MESSAGE(f2(), ".* v1.transpose.. = 1 2 3, details = .*");
}

enum class TestEnum { kFirst, kSecond };
std::ostream& operator<<(std::ostream& out, const TestEnum& e) {
  switch (e) {
    case TestEnum::kFirst:
      out << "first";
      break;
    case TestEnum::kSecond:
      out << "second";
      break;
  }
  return out;
}

GTEST_TEST(DrakeThrowTest, ThrowWithStreamFormat) {
  const TestEnum e1{TestEnum::kFirst};
  const TestEnum e2{TestEnum::kSecond};

  using drake::fmt_streamed;

  auto f1 = [&]() {
    DRAKE_THROW_UNLESS(false, fmt_streamed(e1));
  };
  DRAKE_EXPECT_THROWS_MESSAGE(f1(), ".* e1 = first.*");

  auto f2 = [&]() {
    DRAKE_THROW_UNLESS(false, fmt_streamed(e2));
  };
  DRAKE_EXPECT_THROWS_MESSAGE(f2(), ".* e2 = second.*");
}

}  // namespace