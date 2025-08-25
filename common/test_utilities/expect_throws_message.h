#pragma once

#include <regex>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"

// TODO(sherm1) Add unit tests for these macros. See issue #8403.

#ifdef DRAKE_DOXYGEN_CXX
/** Unit test helper macro for "expecting" an exception to be thrown but also
testing the error message against a provided regular expression. This is
like GTest's `EXPECT_THROW` but does not allow for tested the exception subtype
(because checking the message should suffice on its own).
Usage example: @code
  DRAKE_EXPECT_THROWS_MESSAGE(
      StatementUnderTest(), // You expect this statement to throw ...
      ".*some important.*phrases.*that must appear.*");  // ... this message.
@endcode
The regular expression must match the entire error message. If there is
boilerplate you don't care to match at the beginning and end, surround with
`.*` to ignore in single-line messages or `[\s\S]*` for multiline
messages.

Following GTest's conventions, failure to perform as expected here is a
non-fatal test error. An `ASSERT` variant is provided to make it fatal. There
are also `*_IF_ARMED` variants. These require an exception in Debug builds (or
any builds where `DRAKE_ENABLE_ASSERTS` has been defined). In Release builds,
the expression will pass if it _doesn't_ throw or if it throws an
exception that would pass the same test as in Debug builds. There is no
mechanism for testing _exclusive_ throwing behavior (i.e., only throws in
Debug).
@see DRAKE_ASSERT_THROWS_MESSAGE
@see DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED, DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED
*/
#define DRAKE_EXPECT_THROWS_MESSAGE(expression, regexp)

/** Fatal error version of `DRAKE_EXPECT_THROWS_MESSAGE`.
@see DRAKE_EXPECT_THROWS_MESSAGE */
#define DRAKE_ASSERT_THROWS_MESSAGE(expression, regexp)

/** Same as `DRAKE_EXPECT_THROWS_MESSAGE` in Debug builds, but doesn't _require_
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `DRAKE_ENABLE_ASSERTS` is defined, which Debug builds do be default.
@see DRAKE_EXPECT_THROWS_MESSAGE */
#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, regexp)

/** Same as `DRAKE_ASSERT_THROWS_MESSAGE` in Debug builds, but doesn't require
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `DRAKE_ENABLE_ASSERTS` is defined, which Debug builds do by default.
@see DRAKE_ASSERT_THROWS_MESSAGE */
#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, regexp)

#else  // DRAKE_DOXYGEN_CXX

#define DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, regexp, must_throw, \
                                           fatal_failure)                  \
  do {                                                                     \
    try {                                                                  \
      static_cast<void>(expression);                                       \
      if (must_throw) {                                                    \
        std::string message = "\tExpected: " #expression                   \
                              " throws an exception.\n"                    \
                              " Actual: it throws nothing";                \
        if (fatal_failure) {                                               \
          GTEST_FATAL_FAILURE_(message.c_str());                           \
        } else {                                                           \
          GTEST_NONFATAL_FAILURE_(message.c_str());                        \
        }                                                                  \
      }                                                                    \
    } catch (const std::exception& err) {                                  \
      auto matcher = [](const char* s, const std::string& re) {            \
        return std::regex_match(s, std::regex(re));                        \
      };                                                                   \
      if (fatal_failure) {                                                 \
        ASSERT_PRED2(matcher, err.what(), regexp);                         \
      } else {                                                             \
        EXPECT_PRED2(matcher, err.what(), regexp);                         \
      }                                                                    \
    } catch (...) {                                                        \
      std::string message = "\tExpected: " #expression                     \
                            " throws an exception.\n"                      \
                            " Actual: it throws a non-exception object.";  \
      if (fatal_failure) {                                                 \
        GTEST_FATAL_FAILURE_(message.c_str());                             \
      } else {                                                             \
        GTEST_NONFATAL_FAILURE_(message.c_str());                          \
      }                                                                    \
    }                                                                      \
  } while (0)

#define DRAKE_EXPECT_THROWS_MESSAGE(expression, regexp)                       \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, regexp, true /*must_throw*/, \
                                     false /*non-fatal*/)

#define DRAKE_ASSERT_THROWS_MESSAGE(expression, regexp)                       \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, regexp, true /*must_throw*/, \
                                     true /*fatal*/)

#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, regexp)       \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(                                  \
      expression, regexp, ::drake::kDrakeAssertIsArmed /*must_throw*/, \
      false /*non-fatal*/)

#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, regexp)       \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(                                  \
      expression, regexp, ::drake::kDrakeAssertIsArmed /*must_throw*/, \
      true /*fatal*/)

#endif  // DRAKE_DOXYGEN_CXX
