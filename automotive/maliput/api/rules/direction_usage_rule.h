#pragma once

#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

// TODO(andrew.best@tri.global): Add support for multiple states. Currently,
// it's enforced that all rules have exactly one state and are therefore
// static. To support multiple states, a StateProvider is needed.
/// Rule describing direction usage for a road lane.
///
/// DirectionUsageRules are comprised of:
/// * a zone (a LaneSRange) which specifies the longitudinal section of the
/// road-network to which the rule instance applies.
/// * a catalog of one or more States, each of which indicate the possible
///   DirectionUsageRule semantics for a vehicle traversing the zone.
///
/// A rule instance with a single State is considered "static", and has fixed
/// semantics.
///
/// Each Lane location can be governed by at most one DirectionUsageRule.
class DirectionUsageRule final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirectionUsageRule);

  using Id = TypeSpecificIdentifier<class DirectionUsageRule>;

  /// Semantic state of the DirectionUsageRule.
  /// A State describes the current usage semantics of the lane section.
  /// This includes which direction traffic is allowed to travel on the lane
  /// and the severity of this restriction.
  class State final {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    /// Unique identifier for a State
    using Id = TypeSpecificIdentifier<class State>;

    enum class Severity {
      /// No vehicle should travel on this lane in violation of this rule.
      kStrict = 0,
      /// Vehicles should avoid travelling against this rule but certain
      /// exceptions apply. E.g. passing is allowed.
      kPreferred,
    };

    /// Type of allowed travel on the lane. Categorized based on travel with or
    /// against the central axis (+S) of the lane frame.
    enum class Type {
      /// Travel should proceed in the direction of the +S axis.
      kWithS = 0,
      /// Travel should proceed opposite the +S axis direction.
      kAgainstS,
      /// Travel is allowed both with the lane direction(+S) or against it.
      kBidirectional,
      /// Travel is allowed both with the lane direction(+S) or against it but
      /// should be limited in duration, e.g. when approaching turns.
      kBidirectionalTurnOnly,
      /// Travel on this lane is prohibited.
      kNoUse,
      /// This lane is used to define a parking area.
      kParking,
    };

    /// Constructs a State instance.
    ///
    /// @param id the unique Id
    /// @param type the semantic Type
    /// @param severity the Severity of the State
    State(Id id, Type type, Severity severity)
        : id_(id), type_(type), severity_(severity) {}

    /// Returns the Id.
    const Id& id() const { return id_; }

    /// Returns the Type.
    Type type() const { return type_; }

    /// Returns the Severity.
    Severity severity() const { return severity_; }

   private:
    Id id_;
    Type type_{};
    Severity severity_{};
  };

  // TODO(andrew.best@tri.global): Enable StateProvider and remove the one state
  //                               restriction.
  /// Constructs a DirectionUsageRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param states a vector of valid states for the rule
  /// @throws std::exception if size of states is not exactly 1.
  DirectionUsageRule(const Id& id, const LaneSRange& zone,
                     std::vector<State> states)
      : id_(id), zone_(zone) {
    DRAKE_THROW_UNLESS(states.size() == 1);
    for (const State& state : states) {
      // Construct index of states by ID, ensuring uniqueness of ID's.
      auto result = states_.emplace(state.id(), state);
      DRAKE_THROW_UNLESS(result.second);
    }
  }

  /// Returns the persistent identifier.
  Id id() const { return id_; }

  /// Returns the zone to which this rule instance applies.
  const LaneSRange& zone() const { return zone_; }

  /// Returns the catalog of possible States.
  const std::unordered_map<State::Id, State>& states() const { return states_; }

  /// Returns true if the rule is static, i.e. has only one state,
  /// otherwise false.
  bool is_static() const { return states_.size() == 1; }

  /// Returns the static state of the rule.
  ///
  /// This is a convenience function for returning a static rule's single state.
  ///
  /// @throws std::exception if `is_static()` is false.
  const State& static_state() const {
    DRAKE_THROW_UNLESS(is_static());
    return states_.begin()->second;
  }

  /// Maps DirectionUsageRule::State::Type enums to string representations.
  static std::unordered_map<State::Type, const char*, DefaultHash>
  StateTypeMapper();

 private:
  Id id_;
  LaneSRange zone_;
  std::unordered_map<State::Id, State> states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
