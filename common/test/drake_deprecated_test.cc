#include "drake/common/drake_deprecated.h"

/* clang-format off to disable clang-format-includes */
#include "drake/common/text_logging.h"
/* clang-format on */

#ifdef TEXT_LOGGING_TEST_SPDLOG
#include "drake/common/text_logging_spdlog.h"
#endif

#include <memory>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#ifdef TEXT_LOGGING_TEST_SPDLOG
#include <spdlog/sinks/ostream_sink.h>
#endif

#include "drake/common/test_utilities/expect_throws_message.h"

// This test verifies that the Drake build still succeeds if a deprecated class
// or function is in use.

// The BUILD.bazel rules must supply this flag.  This test code is compiled and
// run twice -- once with spdlog, and once without.
#ifndef TEXT_LOGGING_TEST_SPDLOG
#error Missing a required definition to compile this test case.
#endif

// Check for the expected HAVE_SPDLOG value.
// clang-format off
#if TEXT_LOGGING_TEST_SPDLOG
  #ifndef HAVE_SPDLOG
    #error Missing HAVE_SPDLOG.
  #endif
#else
  #ifdef HAVE_SPDLOG
    #error Unwanted HAVE_SPDLOG.
  #endif
#endif
// clang-format on

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

class DrakeDeprecatedEnvTest : public ::testing::Test {
 protected:
  static constexpr char dep_is_error_env_name[] =
      "DRAKE_ENV_DEPRECATION_IS_ERROR";
  static constexpr char ignore_dep_env_name[] = "DRAKE_ENV_IGNORE_DEPRECATED";

#if TEXT_LOGGING_TEST_SPDLOG
  spdlog::sinks::dist_sink_mt* dist_sink;
  std::vector<spdlog::sink_ptr> original_sub_sinks;
  std::ostringstream stream;

  void SetUp() override {
    dist_sink = drake::logging::get_dist_sink();
    if (dist_sink) {
      auto custom_sink = std::make_shared<spdlog::sinks::ostream_sink_st>(
          stream, true /* flush */);
      dist_sink->set_sinks({custom_sink});
    }
  }

  void TearDown() override {
    if (dist_sink) {
      dist_sink->set_sinks(original_sub_sinks);
    }
  }
#endif
};

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
TEST_F(DrakeDeprecatedEnvTest, WarnOnceTest) {
  static const drake::internal::WarnDeprecated warn_once(
      "2038-01-19", "The method OldCalc() has been renamed to NewCalc().");
#if TEXT_LOGGING_TEST_SPDLOG
  ASSERT_THAT(stream.str(),
              testing::EndsWith("[console] [warning] DRAKE DEPRECATED: "
                                "The method OldCalc() has "
                                "been renamed to NewCalc(). The deprecated "
                                "code will be removed from Drake on or after "
                                "2038-01-19.\n"));
#endif
}

// When the DRAKE_ENV_DEPRECATION_IS_ERROR environment variable is set,
// warnings become errors.
TEST_F(DrakeDeprecatedEnvTest, WarnThrowsTest) {
  ASSERT_EQ(::setenv(dep_is_error_env_name, "1", 1), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      drake::internal::WarnDeprecated("2038-01-19", "Hello"),
      "DRAKE DEPRECATED: Hello. The deprecated code will be removed from Drake "
      "on or after 2038-01-19.");
  ASSERT_EQ(::unsetenv(dep_is_error_env_name), 0);
}

// When DRAKE_ENV_IGNORE_DEPRECATED is set, deprecation warnings are ignored.
TEST_F(DrakeDeprecatedEnvTest, IgnoreWarningsTest) {
  ASSERT_EQ(::setenv(ignore_dep_env_name, "1", 1), 0);
  drake::internal::WarnDeprecated("2038-01-19", "Hello");
#if TEXT_LOGGING_TEST_SPDLOG
  ASSERT_THAT(stream.str(), testing::IsEmpty());
#endif
  ASSERT_EQ(::unsetenv(dep_is_error_env_name), 0);
}

// Check that an exception is thrown when
// DRAKE_ENV_IGNORE_DEPRECATED and DRAKE_ENV_DEPRECATION_IS_ERROR
// are both set.
TEST_F(DrakeDeprecatedEnvTest, InvalidEnvThrows) {
  ASSERT_EQ(::setenv(dep_is_error_env_name, "1", 1), 0);
  ASSERT_EQ(::setenv(ignore_dep_env_name, "1", 1), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      drake::internal::WarnDeprecated("2038-01-19", "Hello"),
      "DRAKE_ENV_DEPRECATION_IS_ERROR and DRAKE_ENV_IGNORE_DEPRECATED cannot "
      "both be set to \"1\"");
  ASSERT_EQ(::unsetenv(dep_is_error_env_name), 0);
  ASSERT_EQ(::unsetenv(ignore_dep_env_name), 0);
}

}  // namespace
