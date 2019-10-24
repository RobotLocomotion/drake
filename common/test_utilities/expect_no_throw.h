#pragma once

#include <exception>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

// Private helper macro to ensure error messages are the same for EXPECT and
// ASSERT variants of this macro.
#define _DRAKE_EXPECT_NO_THROW_HELPER(stmt, fail_macro) \
  try { \
    (void)(stmt); \
  } catch (const std::exception& e) { \
    fail_macro() \
      << "Expected: Does not throw:\n  " << #stmt << std::endl \
      << "Actual: Throws " << ::drake::NiceTypeName::Get(e) << std::endl \
      << "  " << e.what(); \
  } catch (...) { \
    fail_macro() \
      << "Expected: Does not throw:\n  " << #stmt << std::endl \
      << "Actual: Throws type which does not inherit from std::exception"; \
  }

/**
Unittest helper to explicitly indicate that a statement should not throw an
exception. This should be used in place of EXPECT_NO_THROW because it will have
useful information when a failure indeed happens.
*/
#define DRAKE_EXPECT_NO_THROW(stmt) \
  _DRAKE_EXPECT_NO_THROW_HELPER(stmt, ADD_FAILURE)

/**
Same as DRAKE_EXPECT_NO_THROW, but halts the execution of the given test case
on failure.
*/
#define DRAKE_ASSERT_NO_THROW(stmt) \
  _DRAKE_EXPECT_NO_THROW_HELPER(stmt, GTEST_FAIL)
