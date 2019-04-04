#include "drake/automotive/maliput/api/rules/phase.h"

#include <utility>

namespace drake {
namespace maliput {
namespace api {
namespace rules {

Phase::Phase(const Id& id, const RuleStates& rule_states,
             optional<BulbStates> bulb_states)
    : id_(id),
      rule_states_(std::move(rule_states)),
      bulb_states_(std::move(bulb_states)) {}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
