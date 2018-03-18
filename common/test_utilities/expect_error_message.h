#pragma once

#include <regex>
#include <string>

// Helper macro for "expecting" an exception but *also* testing the error
// message against the provided regular expression.
#define DRAKE_EXPECT_ERROR_MESSAGE(expression, exception, reg_exp) \
try { \
  expression; \
  GTEST_NONFATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const std::string& re) { \
    return std::regex_match(s, std::regex(re)); }; \
  EXPECT_PRED2(matcher, err.what(), reg_exp); \
}

// Helper macro for "asserting" an exception but *also* testing the error
// message against the provided regular expression.
#define DRAKE_ASSERT_ERROR_MESSAGE(expression, exception, reg_exp) \
try { \
  expression; \
  GTEST_FATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const std::string& re) { \
    return std::regex_match(s, std::regex(re)); }; \
  ASSERT_PRED2(matcher, err.what(), reg_exp); \
}

// Helper macros that mirror the above macros, except they depend on
// whether assert is armed or not. If *not* armed, they become EXPECT_NO_THROW.
#ifdef DRAKE_ASSERT_IS_DISARMED

#define DRAKE_EXPECT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
do { \
  EXPECT_NO_THROW(expression); \
} while (0)

#define DRAKE_ASSERT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
do { \
  EXPECT_NO_THROW(expression); \
} while (0)

#else

#define DRAKE_EXPECT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
DRAKE_EXPECT_ERROR_MESSAGE(expression, exception, regexp)

#define DRAKE_ASSERT_ERROR_MESSAGE_IF_ARMED(expression, exception, regexp) \
DRAKE_ASSERT_ERROR_MESSAGE(expression, exception, regexp)

#endif
