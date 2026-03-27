#include "drake/common/drake_deprecated.h"

#include <memory>
#include <sstream>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_spdlog.h"

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

class DrakeDeprecatedEnvTest : public ::testing::Test {
 protected:
  static constexpr char kEnvSeverity[] = "DRAKE_DEPRECATION_RUNTIME_SEVERITY";

  spdlog::sinks::dist_sink_mt* dist_sink_{};
  std::vector<spdlog::sink_ptr> original_sub_sinks_;
  std::ostringstream stream_;

  void SetUp() override {
    dist_sink_ = drake::logging::get_dist_sink();
    if (dist_sink_) {
      original_sub_sinks_ = dist_sink_->sinks();
      auto custom_sink = std::make_shared<spdlog::sinks::ostream_sink_st>(
          stream_, true /* flush */);
      dist_sink_->set_sinks({custom_sink});
    }
  }

  void TearDown() override {
    ::unsetenv(kEnvSeverity);
    if (dist_sink_) {
      dist_sink_->set_sinks(original_sub_sinks_);
    }
  }
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

// Check that the "warn once" idiom compiles and warns.
TEST_F(DrakeDeprecatedEnvTest, WarnOnceTest) {
  static const drake::internal::WarnDeprecated warn_once(
      "2038-01-19", "The method OldCalc() has been renamed to NewCalc().");

  ASSERT_THAT(
      stream_.str(),
      testing::EndsWith(
          "[console] [warning] DRAKE DEPRECATED: The method OldCalc() has "
          "been renamed to NewCalc(). The deprecated code will be removed "
          "from Drake on or after 2038-01-19.\n"));
}

// When DRAKE_DEPRECATION_RUNTIME_SEVERITY=error, warnings become errors.
TEST_F(DrakeDeprecatedEnvTest, WarnThrowsTest) {
  ASSERT_EQ(::setenv(kEnvSeverity, "error", 1), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      drake::internal::WarnDeprecated("2038-01-19", "Hello"),
      "DRAKE DEPRECATED: Hello. The deprecated code will be removed from Drake "
      "on or after 2038-01-19.");
}

// When DRAKE_DEPRECATION_RUNTIME_SEVERITY=ignore, warnings are suppressed.
TEST_F(DrakeDeprecatedEnvTest, IgnoreWarningsTest) {
  ASSERT_EQ(::setenv(kEnvSeverity, "ignore", 1), 0);
  drake::internal::WarnDeprecated("2038-01-19", "Hello");
  ASSERT_THAT(stream_.str(), testing::IsEmpty());
}

// When DRAKE_DEPRECATION_RUNTIME_SEVERITY is an unrecognized value, a warning
// is logged about the bad value and the deprecation warning is still shown.
// The invalid-env warning is only logged once per process, not on every call.
TEST_F(DrakeDeprecatedEnvTest, InvalidEnvWarns) {
  ASSERT_EQ(::setenv(kEnvSeverity, "bad_value", 1), 0);
  // First call: both the invalid-env warning and the deprecation warning
  // appear.
  drake::internal::WarnDeprecated("2038-01-19", "Hello");
  ASSERT_THAT(stream_.str(),
              testing::HasSubstr("unrecognized value \"bad_value\"."));
  ASSERT_THAT(stream_.str(), testing::HasSubstr("DEPRECATED: Hello."));
  // Second call: the invalid-env warning must not repeat.
  stream_.str("");
  drake::internal::WarnDeprecated("2038-01-19", "Hello again");
  ASSERT_THAT(
      stream_.str(),
      testing::Not(testing::HasSubstr("DRAKE_DEPRECATION_RUNTIME_SEVERITY")));
  ASSERT_THAT(stream_.str(), testing::HasSubstr("DEPRECATED: Hello again."));
}

}  // namespace
