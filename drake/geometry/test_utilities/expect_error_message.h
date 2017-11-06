#pragma once

#include <regex>

// Helper macro for "expecting" an exception but *also* testing the error
// message against the provided regular expression.
#define EXPECT_ERROR_MESSAGE(expression, exception, reg_exp) \
try { \
  expression; \
  GTEST_NONFATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const char* re) { \
    return std::regex_match(s, std::regex(re)); }; \
  EXPECT_PRED2(matcher, err.what(), reg_exp); \
}

// Helper macro for "asserting" an exception but *also* testing the error
// message against the provided regular expression.
#define ASSERT_ERROR_MESSAGE(expression, exception, reg_exp) \
try { \
  expression; \
  GTEST_FATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const char* re) { \
    return std::regex_match(s, std::regex(re)); }; \
  ASSERT_PRED2(matcher, err.what(), reg_exp); \
}

// Helper macros for which mirror the above macros, except they depend on
// whether assert is armed or not. If *not* armed, they become EXPECT_NO_THROW.
#ifdef DRAKE_ASSERT_IS_DISARMED

#define EXPECT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
do {\
  EXPECT_NO_THROW(expression); \
} while (0)

#define ASSERT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
do {\
  EXPECT_NO_THROW(expression); \
} while (0)

#else

#define EXPECT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
EXPECT_ERROR_MESSAGE(expression, exception, regexp)

#define ASSERT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
ASSERT_ERROR_MESSAGE(expression, exception, regexp)

#endif
