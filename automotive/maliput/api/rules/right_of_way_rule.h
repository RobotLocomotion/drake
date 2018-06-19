#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {


/// Rule describing right-of-way, a.k.a. priority.
///
/// Right-of-way rules cover things like stop signs, yield signs, and
/// traffic lights:  in other words, control over how competing traffic
/// flows take turns traversing regions of the road network.
///
/// Each rule instance comprises:
/// * a zone (a LaneSRoute) which specifies a contiguous longitudinal
///   lane-wise section of the road network to which the rule instance
///   applies;
/// * a ZoneType describing whether or not stopping within the zone is
///   allowed;
/// * a catalog of one or more States, each of which indicate the possible
///   right-of-way semantics for a vehicle traversing the zone.
///
/// The `zone` is directed; the rule applies to vehicles traveling forward
/// through the `zone`.
///
/// A rule instance with a single State is considered "static", and has fixed
/// semantics.  A rule instance with multiple States is considered "dynamic"
/// and determination of the active rule State at any given time is delegated
/// to a RightOfWayStateProvider agent, linked by the rule's Id.
class RightOfWayRule final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RightOfWayRule);

  /// Unique identifier for a RightOfWayRule.
  using Id = TypeSpecificIdentifier<class RightOfWayRule>;

  /// Description of stopping properties of the zone.
  enum class ZoneType {
    kStopExcluded,  ///< Vehicles should not stop within the zone; vehicles
                    ///  should avoid entering the zone if traffic conditions
                    ///  may cause them to stop within the zone.  Vehicles
                    ///  already in the zone when a kStop state occurs should
                    ///  exit the zone.
    kStopAllowed    ///< Vehicles are allowed to stop within the zone.
  };

  /// Semantic state of a RightOfWayRule.
  ///
  /// A State describes the semantic state of a RightOfWayRule,
  /// basically "Go", "Stop", or "Stop, Then Go".  A RightOfWayRule
  /// may have multiple possible States, in which case its States must
  /// have Id's which are unique within the context of that
  /// RightOfWayRule.
  ///
  /// A State also describes the yield logic of a RightOfWayRule, via
  /// a list of Id's of other RightOfWayRules (and thus the zones
  /// which they control) which have priority over this rule.  An
  /// empty list means that a rule in this State has priority over all
  /// other rules.  Vehicles with lower priority (i.e., traveling on
  /// lower-priority paths) must yield to vehicles with higher priority.
  class State final {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    /// Unique identifier for a State
    using Id = TypeSpecificIdentifier<class State>;

    /// List of RightOfWayRule::Id's of rules/zones with priority.
    using YieldGroup = std::vector<RightOfWayRule::Id>;

    /// Basic semantic type of a rule state.
    enum class Type {
      kGo = 0,              ///< Vehicle has right-of-way and may proceed if
                            ///  safe to do so.
      kStop,                ///< Vehicle does not have right-of-way and must
                            ///  stop.
      kStopThenGo           ///< Vehicle must come to complete stop before
                            ///  entering controlled zone, but may then
                            ///  proceed if safe;
    };

    /// Constructs a State instance.
    ///
    /// @param id the unique Id
    /// @param type the semantic Type
    /// @param yield_to the other paths/rules which must be yielded to
    State(Id id, Type type, const YieldGroup& yield_to)
        : id_(id), type_(type), yield_to_(yield_to) {}

    /// Returns the Id.
    const Id& id() const { return id_; }

    /// Returns the Type.
    Type type() const { return type_; }

    /// Returns the YieldGroup.
    const YieldGroup& yield_to() const { return yield_to_; }

   private:
    Id id_;
    Type type_{};
    YieldGroup yield_to_;
  };

  /// Constructs a RightOfWayRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param controlled_zone LaneSRoute to which this rule applies
  /// @param type the static semantics of this rule
  ///
  /// Throws a std::exception if `states` is empty or if `states` contains
  /// duplicate State::Id's.
  RightOfWayRule(const Id& id,
                 const LaneSRoute& zone,
                 ZoneType zone_type,
                 const std::vector<State>& states)
      : id_(id), zone_(zone), zone_type_(zone_type) {
    DRAKE_THROW_UNLESS(states.size() >= 1);
    for (const State& state : states) {
      // Construct index of states by ID, ensuring uniqueness of ID's.
      auto result = states_.emplace(state.id(), state);
      DRAKE_THROW_UNLESS(result.second);
    }
  }

  /// Returns the rule's identifier.
  const Id& id() const { return id_; }

  /// Returns the zone controlled by the rule.
  const LaneSRoute& zone() const { return zone_; }

  /// Returns the zone's type.
  ZoneType zone_type() const { return zone_type_; }

  /// Returns the catalog of possible States.
  const std::unordered_map<State::Id, State>& states() const { return states_; }

  /// Returns true if the rule is static, i.e., has no dynamic state,
  /// otherwise false.
  ///
  /// This is true if and only if the rule has a single state.
  bool is_static() const { return states_.size() == 1; }

  /// Returns the static state of the rule.
  ///
  /// This is a convenience function for returning a static rule's single state.
  ///
  /// Throws a std::exception if `is_static()` is false.
  const State& static_state() const {
    DRAKE_THROW_UNLESS(is_static());
    return states_.begin()->second;
  }

 private:
  Id id_;
  LaneSRoute zone_;
  ZoneType zone_type_{};
  std::unordered_map<State::Id, State> states_;
};


/// Abstract interface for the provider of the state of a dynamic
/// (multiple state) RightOfWayRule.
class RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayStateProvider)

  virtual ~RightOfWayStateProvider() = default;

  /// Result returned by GetState().
  struct Result {
    /// Information about a subsequent State.
    struct Next {
      /// ID of the State.
      RightOfWayRule::State::Id id;
      /// If known, estimated time until the transition to the State.
      drake::optional<double> duration_until;
    };

    /// ID of the rule's current State.
    RightOfWayRule::State::Id current_id;
    /// Information about the rule's upcoming State if a state transition
    /// is anticipated.
    drake::optional<Next> next;
  };

  /// Gets the state of the RightOfWayRule identified by `id`.
  ///
  /// Returns a Result struct bearing the State::Id of the rule's current
  /// state.  If a transition to a new state is anticipated,
  /// Result::next will be populated and bear the State::Id of the next
  /// state.  If the time until the transition is known, then
  /// Result::next.duration_until will be populated with that duration.
  ///
  /// Returns nullopt if `id` is unrecognized, which would be the case
  /// if no such rule exists or if the rule has only static semantics.
  drake::optional<Result> GetState(const RightOfWayRule::Id& id) const {
    return DoGetState(id);
  }

 protected:
  RightOfWayStateProvider() = default;

 private:
  virtual drake::optional<Result> DoGetState(
      const RightOfWayRule::Id& id) const = 0;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
