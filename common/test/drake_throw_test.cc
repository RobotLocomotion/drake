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
//   1. Provided value expressions get encoded as expected.
//   2. Regression guaranteeing that four value expressions are supported.
//
// If we have hiccups with value expressions of various types, these tests can
// be extended to explicitly test those types.
GTEST_TEST(DrakeThrowTest, ThrowWithValues) {
  // Create a collection of lvalues.
  const std::string msg = "a string";
  const double d = 7;
  const int i = 13;
  const char bytes[] = "beef";
  // const Eigen::Vector3d bytes(1, 2, 3);

  // Ensure it works with up to four values.
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d, i));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d, i, msg));
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true, d, i, msg, bytes));

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
        DRAKE_THROW_UNLESS(false, d, msg, i);
        return;
      case 4:
        DRAKE_THROW_UNLESS(false, d, msg, i, bytes);
        return;
    }
    DRAKE_UNREACHABLE();
  };

  DRAKE_EXPECT_THROWS_MESSAGE(f1(1), ".*d = 7.");
  DRAKE_EXPECT_THROWS_MESSAGE(f1(2), ".*d = 7, msg = \"a string\".");
  DRAKE_EXPECT_THROWS_MESSAGE(f1(3), ".*d = 7, msg = \"a string\", i = 13.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      f1(4), ".*d = 7, msg = \"a string\", i = 13, bytes = \"beef\".");
}

GTEST_TEST(DrakeThrowTest, ThrowWithEigenValues) {
  const std::string details = "These are details";
  const Eigen::Vector3d v1(1, 2, 3);

  auto f1 = [&]() {
    DRAKE_THROW_UNLESS(false, v1, details);
  };
  DRAKE_EXPECT_THROWS_MESSAGE(f1(), ".*v1 = 1\n2\n3, details = .*");

  auto f2 = [&]() {
    DRAKE_THROW_UNLESS(false, v1.transpose(), details);
  };
  DRAKE_EXPECT_THROWS_MESSAGE(f2(), ".*v1.transpose.. = 1 2 3, details = .*");
}

}  // namespace
