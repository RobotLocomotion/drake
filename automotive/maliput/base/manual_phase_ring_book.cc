#include "drake/automotive/maliput/base/manual_phase_ring_book.h"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {

using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::RightOfWayRule;

class ManualPhaseRingBook::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() {}
  ~Impl() {}

  void AddPhaseRing(const PhaseRing& ring) {
    auto result = ring_book_.emplace(ring.id(), ring);
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple PhaseRing instances "
                             "with ID " + ring.id().string());
    }
    const Phase& phase = ring.phases().begin()->second;
    for (const auto& element : phase.rule_states()) {
      auto r = rule_book_.emplace(element.first, ring.id());
      if (!r.second) {
        throw std::logic_error("RightOfWayRule with ID " +
                               element.first.string() + " is part of more than "
                               "one PhaseRing.");
      }
    }
  }

  void RemovePhaseRing(const PhaseRing::Id& ring_id) {
    const optional<PhaseRing> ring = DoGetPhaseRing(ring_id);
    if (ring == nullopt) {
      throw std::logic_error("Attempted to remove unknown PhaseRing with ID " +
                             ring_id.string());
    }
    DRAKE_THROW_UNLESS(ring_book_.erase(ring_id) == 1);
    const Phase& phase = ring->phases().begin()->second;
    for (const auto& element : phase.rule_states()) {
      DRAKE_THROW_UNLESS(rule_book_.erase(element.first) == 1);
    }
  }

  std::vector<PhaseRing::Id> DoGetPhaseRings() const {
    std::vector<PhaseRing::Id> result;
    result.reserve(ring_book_.size());
    for (const auto& element : ring_book_) {
      result.push_back(element.first);
    }
    return result;
  }

  optional<PhaseRing> DoGetPhaseRing(const PhaseRing::Id& ring_id) const {
    auto it = ring_book_.find(ring_id);
    if (it == ring_book_.end()) {
      return nullopt;
    }
    return it->second;
  }

  optional<PhaseRing> DoFindPhaseRing(const RightOfWayRule::Id& rule_id) const {
    auto it = rule_book_.find(rule_id);
    if (it == rule_book_.end()) {
      return nullopt;
    }
    return ring_book_.at(it->second);
  }

 private:
  std::unordered_map<PhaseRing::Id, const PhaseRing> ring_book_;
  std::unordered_map<RightOfWayRule::Id, const PhaseRing::Id> rule_book_;
};

ManualPhaseRingBook::ManualPhaseRingBook()
    : impl_(std::make_unique<Impl>()) {}

ManualPhaseRingBook::~ManualPhaseRingBook() = default;

void ManualPhaseRingBook::AddPhaseRing(const PhaseRing& ring) {
  impl_->AddPhaseRing(ring);
}

void ManualPhaseRingBook::RemovePhaseRing(const PhaseRing::Id& ring_id) {
  impl_->RemovePhaseRing(ring_id);
}

std::vector<PhaseRing::Id> ManualPhaseRingBook::DoGetPhaseRings() const {
  return impl_->DoGetPhaseRings();
}

optional<PhaseRing> ManualPhaseRingBook::DoGetPhaseRing(
    const PhaseRing::Id& ring_id) const {
  return impl_->DoGetPhaseRing(ring_id);
}

optional<PhaseRing> ManualPhaseRingBook::DoFindPhaseRing(
    const RightOfWayRule::Id& rule_id) const {
  return impl_->DoFindPhaseRing(rule_id);
}

}  // namespace maliput
}  // namespace drake
