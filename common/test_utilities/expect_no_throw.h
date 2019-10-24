#pragma once

#include <exception>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

/**
Unittest helper to explicitly indicate that a statement should not throw an
error. This should be used in place of EXPECT_NO_THROW because it will have
useful information when a failure indeed happens.
*/
#define DRAKE_ASSERT_NO_THROW(stmt)    (void)(stmt)

/**
Same as DRAKE_ASSERT_NO_THROW, but does not halt execution of the given test
case.
*/
#define DRAKE_EXPECT_NO_THROW(stmt) \
  try { \
    (void)(stmt); \
  } catch (const std::exception& e) { \
    ADD_FAILURE() \
      << "Expected: Does not throw:\n  " << #stmt << std::endl \
      << "Actual: Throws " << ::drake::NiceTypeName::Get(e) << std::endl \
      << "  " << e.what(); \
  } catch (...) { \
    ADD_FAILURE() \
      << "Expected: Does not throw:\n  " << #stmt << std::endl \
      << "Actual: Throws unknown exception type"; \
  }
