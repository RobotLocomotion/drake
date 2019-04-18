#include "drake/automotive/maliput/base/manual_phase_provider.h"

#include <stdexcept>
#include <string>
#include <unordered_map>

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {

using api::rules::PhaseProvider;

class ManualPhaseProvider::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() {}
  ~Impl() {}

  void AddPhaseRing(const api::rules::PhaseRing::Id& id,
                    const api::rules::Phase::Id& initial_phase) {
    auto result = phases_.emplace(id, initial_phase);
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple phase rings with id " +
                             id.string());
    }
  }

  void SetPhase(const api::rules::PhaseRing::Id& id,
                const api::rules::Phase::Id& phase) {
    phases_.at(id) = phase;
  }

  optional<PhaseProvider::Result> DoGetPhase(
      const api::rules::PhaseRing::Id& id) const {
    auto it = phases_.find(id);
    if (it == phases_.end()) {
      return nullopt;
    }
    // TODO(liang.fok) Add support for "next phase" and "duration until", #9993.
    return PhaseProvider::Result{it->second, nullopt};
  }

 private:
  std::unordered_map<maliput::api::rules::PhaseRing::Id,
                     maliput::api::rules::Phase::Id>
      phases_;
};

ManualPhaseProvider::ManualPhaseProvider() : impl_(std::make_unique<Impl>()) {}

ManualPhaseProvider::~ManualPhaseProvider() = default;

void ManualPhaseProvider::AddPhaseRing(
    const api::rules::PhaseRing::Id& id,
    const api::rules::Phase::Id& initial_phase) {
  impl_->AddPhaseRing(id, initial_phase);
}

void ManualPhaseProvider::SetPhase(const api::rules::PhaseRing::Id& id,
                                   const api::rules::Phase::Id& phase) {
  impl_->SetPhase(id, phase);
}

optional<PhaseProvider::Result> ManualPhaseProvider::DoGetPhase(
    const api::rules::PhaseRing::Id& id) const {
  return impl_->DoGetPhase(id);
}

}  // namespace maliput
}  // namespace drake
