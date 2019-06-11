#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {


// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
/// Predicate-formatter which tests equality of SpeedLimitRule::Severity.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::SpeedLimitRule::Severity a,
                                   rules::SpeedLimitRule::Severity b);


/// Predicate-formatter which tests equality of SpeedLimitRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::SpeedLimitRule& a,
                                   const rules::SpeedLimitRule& b);


}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
