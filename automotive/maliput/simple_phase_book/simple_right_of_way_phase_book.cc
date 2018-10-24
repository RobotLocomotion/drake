#include "drake/automotive/maliput/simple_phase_book/simple_right_of_way_phase_book.h"

#include <stdexcept>
#include <string>

namespace drake {
namespace maliput {
namespace simple_phase_book {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayPhaseRing;

void SimpleRightOfWayPhaseBook::AddEntry(
    const RightOfWayRule::Id& rule_id, const RightOfWayPhaseRing::Id& ring_id) {
  auto result = phase_book_.emplace(rule_id, ring_id);
  if (!result.second) {
     throw std::logic_error("Attempted to add multiple RightOfWayRules with "
                            " ID " + rule_id.string());
  }
}

optional<RightOfWayPhaseRing::Id> SimpleRightOfWayPhaseBook::DoGetPhaseRing(
    const RightOfWayRule::Id& id) const {
  auto it = phase_book_.find(id);
  if (it == phase_book_.end()) {
    return nullopt;
  }
  return it->second;
}

}  // namespace simple_phase_book
}  // namespace maliput
}  // namespace drake
