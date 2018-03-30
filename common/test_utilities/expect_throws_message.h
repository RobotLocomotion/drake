#pragma once

#include <regex>
#include <string>

// TODO(sherm1) Add unit tests for these macros. See issue #8403.

#ifdef DRAKE_DOXYGEN_CXX
/** Unit test helper macro for "expecting" an exception to be thrown but also
testing the error message against a provided regular expression. This is
like GTest's `EXPECT_THROW` but is fussier about the particular error message.
Usage example: @code
  DRAKE_EXPECT_THROWS_MESSAGE(
      StatementUnderTest(), // You expect this statement to throw ...
      std::logic_error,     // ... this exception with ...
      ".*some important.*phrases.*that must appear.*");  // ... this message.
@endcode
The regular expression must match the entire error message. If there is
boilerplate you don't care to match at the beginning and end, surround with
`.*` to ignore.

Following GTest's conventions, failure to perform as expected here is a
non-fatal test error. An `ASSERT` variant is provided to make it fatal. Also,
we provide `IF_ARMED` variants for testing error messages that are thrown only
in Debug builds (or any builds where `DRAKE_ENABLE_ASSERTS` has been defined).
@see DRAKE_ASSERT_THROWS_MESSAGE
@see DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED, DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED */
#define DRAKE_EXPECT_THROWS_MESSAGE(expression, exception, regexp)

/** Fatal error version of `DRAKE_EXPECT_THROWS_MESSAGE`.
@see DRAKE_EXPECT_THROWS_MESSAGE */
#define DRAKE_ASSERT_THROWS_MESSAGE(expression, exception, regexp)

/** Same as `DRAKE_EXPECT_THROWS_MESSAGE` in Debug builds, but doesn't require
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `DRAKE_ENABLE_ASSERTS` is defined, which Debug builds do be default.
@see DRAKE_EXPECT_THROWS_MESSAGE */
#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp)

/** Same as `DRAKE_ASSERT_THROWS_MESSAGE` in Debug builds, but doesn't require
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `DRAKE_ENABLE_ASSERTS` is defined, which Debug builds do be default.
@see DRAKE_ASSERT_THROWS_MESSAGE */
#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp)
#endif

#ifndef DRAKE_DOXYGEN_CXX
#define DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                           must_throw, fatal_failure) \
try { \
  expression; \
  if (must_throw) { \
    if (fatal_failure) { \
      GTEST_FATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
    } else { \
      GTEST_NONFATAL_FAILURE_("\t" #expression " failed to throw " #exception);\
    } \
  } \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const std::string& re) { \
    return std::regex_match(s, std::regex(re)); }; \
  if (fatal_failure) { \
    ASSERT_PRED2(matcher, err.what(), regexp); \
  } else { \
    EXPECT_PRED2(matcher, err.what(), regexp); \
  } \
}

#define DRAKE_EXPECT_THROWS_MESSAGE(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     true /*must_throw*/, false /*non-fatal*/)

#define DRAKE_ASSERT_THROWS_MESSAGE(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     true /*must_throw*/, true /*fatal*/)

#ifdef DRAKE_ASSERT_IS_DISARMED
// Throwing the expected message is optional in this case.

#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     false /*optional*/, false /*non-fatal*/)

#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     false /*optional*/, true /*fatal*/)

#else
// Throwing the expected message is required in this case.

#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE(expression, exception, regexp)

#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_ASSERT_THROWS_MESSAGE(expression, exception, regexp)

#endif
#endif
