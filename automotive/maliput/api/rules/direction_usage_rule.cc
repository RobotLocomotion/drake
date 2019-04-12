#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

std::unordered_map<DirectionUsageRule::State::Type, const char*, DefaultHash>
DirectionUsageRule::StateTypeMapper() {
  return {{DirectionUsageRule::State::Type::kWithS, "WithS"},
          {DirectionUsageRule::State::Type::kAgainstS, "AgainstS"},
          {DirectionUsageRule::State::Type::kBidirectional, "Bidirectional"},
          {DirectionUsageRule::State::Type::kBidirectionalTurnOnly,
           "BidirectionalTurnOnly"},
          {DirectionUsageRule::State::Type::kNoUse, "NoUse"},
          {DirectionUsageRule::State::Type::kParking, "Parking"}};
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
