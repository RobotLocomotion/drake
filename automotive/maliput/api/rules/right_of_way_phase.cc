#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

RightOfWayPhase::RightOfWayPhase(const Id& id, const RuleStates& rule_states)
    : id_(id), rule_states_(rule_states) {}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
