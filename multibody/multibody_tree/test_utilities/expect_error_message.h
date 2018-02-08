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
