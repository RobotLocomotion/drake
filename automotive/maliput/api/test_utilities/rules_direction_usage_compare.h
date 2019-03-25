#pragma once

#include <unordered_map>

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
// Issue #10950 tracks this improvement.
/// Predicate-formatter which tests equality of DirectionUsageRule::State::Type.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   DirectionUsageRule::State::Type a,
                                   DirectionUsageRule::State::Type b);

/// Predicate-formatter which tests equality of
/// DirectionUsageRule::State::Severity.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   DirectionUsageRule::State::Severity a,
                                   DirectionUsageRule::State::Severity b);

/// Predicate-formatter which tests equality of DirectionUsageRule::State.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const DirectionUsageRule::State& a,
                                   const DirectionUsageRule::State& b);

/// Predicate-formatter which tests equality of DirectionUsageRule::State maps.
::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::unordered_map<DirectionUsageRule::State::Id,
                             DirectionUsageRule::State>& a,
    const std::unordered_map<DirectionUsageRule::State::Id,
                             DirectionUsageRule::State>& b);

/// Predicate-formatter which tests equality of DirectionUsageRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const DirectionUsageRule& a,
                                   const DirectionUsageRule& b);

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
