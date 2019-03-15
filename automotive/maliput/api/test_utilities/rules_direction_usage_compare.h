#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {

// TODO(andrew.best@tri.global)  This should be replaced by a generic predicate
//                               which handles anything with operator==.
/// Predicate-formatter which tests equality of DirectionUsageRule::Severity.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::DirectionUsageRule::Severity a,
                                   rules::DirectionUsageRule::Severity b);

/// Predicate-formatter which tests equality of DirectionUsageRule::Direction.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::DirectionUsageRule::Direction a,
                                   rules::DirectionUsageRule::Direction b);

/// Predicate-formatter which tests equality of DirectionUsageRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::DirectionUsageRule& a,
                                   const rules::DirectionUsageRule& b);

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
