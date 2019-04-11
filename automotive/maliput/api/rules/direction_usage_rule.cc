#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

std::unordered_map<DirectionUsageRule::State::Type, const char*, DefaultHash>
DirectionUsageRule::StateTypeMapper() {
  std::unordered_map<DirectionUsageRule::State::Type, const char*, DefaultHash>
      result;
  result.emplace(DirectionUsageRule::State::Type::kWithS, "WithS");
  result.emplace(DirectionUsageRule::State::Type::kAgainstS, "AgainstS");
  result.emplace(DirectionUsageRule::State::Type::kBidirectional,
                 "Bidirectional");
  result.emplace(DirectionUsageRule::State::Type::kBidirectionalTurnOnly,
                 "BidirectionalTurnOnly");
  result.emplace(DirectionUsageRule::State::Type::kNoUse, "NoUse");
  result.emplace(DirectionUsageRule::State::Type::kParking, "Parking");
  return result;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
