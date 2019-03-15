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
/// Predicate-formatter which tests equality of DirectionUsageRule::State::Type.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::DirectionUsageRule::State::Type a,
                                   rules::DirectionUsageRule::State::Type b);

/// Predicate-formatter which tests equality of
/// DirectionUsageRule::State::Severity.
::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    rules::DirectionUsageRule::State::Severity a,
    rules::DirectionUsageRule::State::Severity b);

/// Predicate-formatter which tests equality of DirectionUsageRule::State.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::DirectionUsageRule::State& a,
                                   const rules::DirectionUsageRule::State& b);

/// Predicate-formatter which tests equality of an unordered_map of
/// DirectionUsageRule::State.
::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::unordered_map<rules::DirectionUsageRule::State::Id,
                       rules::DirectionUsageRule::State>& a,
    const std::unordered_map<rules::DirectionUsageRule::State::Id,
                       rules::DirectionUsageRule::State>& b);

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
