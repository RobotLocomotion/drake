#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {


/// Predicate-formatter which tests equality of RightOfWayRule::Type.
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::RightOfWayRule::Type a,
                                   rules::RightOfWayRule::Type b);


/// Predicate-formatter which tests equality of RightOfWayRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b);


}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
