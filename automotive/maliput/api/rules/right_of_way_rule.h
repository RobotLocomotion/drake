#pragma once

#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

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
/// * a controlled-zone, which specifies a contiguous longitudinal lane-wise
///   section of the road network to which the instance applies;
/// * a type which indicates the right-of-way semantics for a vehicle
///   traversing the controlled-zone.
/// The type may be "dynamic", in which the semantic state is delegated to
/// an RightOfWayStateProvider agent, linked by this rule's Id.
///
/// A RightOfWayRule instance determines whether or not a vehicle is
/// allowed to enter the controlled-zone.  Having entered, vehicles
/// should not stop within a controlled-zone, and thus should not
/// enter a the controlled-zone if traffic conditions may cause them
/// to stop within the zone.
class RightOfWayRule {
 public:
  using Id = TypeSpecificIdentifier<class RightOfWayRule>;

  /// Basic static semantics of the rule instance.
  // TODO(maddog@tri.global) Describe turn-taking rules for StopThenGo
  //                          (e.g., 4-way stop intersection), which is
  //                          related to...
  // TODO(maddog@tri.global) Add explicit yield-to-whom semantics.
  enum class Type {
    kProceed = 0,  ///< No restriction on vehicle's right-of-way, but it
                   ///  must still avoid stopping within the controlled-zone.
    kYield,        ///< Vehicles must yield to competing traffic.
    kStopThenGo,   ///< Vehicles must come to a complete stop before entering
                   ///  the controlled-zone, but may then proceed if safe.
    kDynamic       ///< Delegate semantics to a RightOfWayStateProvider,
                   ///  with state linked by the Id of this rule.
  };

  /// Rule semantics which may be expressed via a dynamic agent.
  // TODO(maddog@tri.global) Better define fallback semantics of "dark-mode".
  // TODO(maddog@tri.global) Better distinguish Go, Caution, Yield
  enum class DynamicState {
    kUncontrolled = 0,    ///< Signaling apparatus is non-functional (e.g.,
                          ///  "dark mode").
    kGo,                  ///< Vehicle has right-of-way.
    kPrepareToStop,       ///< Vehicle has right-of-way, but rule state will
                          ///  soon transition to kStop.
    kStop,                ///< Vehicle does not have right-of-way (and thus
                          ///  must not enter the controlled-zone).
    kPrepareToGo,         ///< Vehicle does not have right-of-way, but rule
                          ///  state will soon transition to kGo.
    kProceedWithCaution,  ///< Vehicle has right-of-way.
    kYield,               ///< Vehicles must yield to competing traffic.
    kStopThenGo           ///< Vehicle must come to complete stop before
                          ///  entering controlled-zone, but may then
                          ///  proceed if safe;
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RightOfWayRule)

  RightOfWayRule(const Id& id, Type type) : id_(id), type_(type) {}

  /// Returns the persistent identifier.
  Id id() const { return id_; }

  // TODO(maddog@tri.global)  Define controlled_zone accessor.

  /// Return the static rule semantic.
  Type type() const { return type_; }

 private:
  Id id_;
  // TODO(maddog@tri.global)  Define controlled_zone as
  //                          LaneRoute controlled_zone_;
  Type type_{};
  // TODO(maddog) Add bool field for "stopping is excluded in zone"?
};


/// Abstract interface for the provider of the dynamic semantic state of
/// a RightOfWayRule.
class RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayStateProvider)

  virtual ~RightOfWayStateProvider() = default;

  /// Return the current state of the RightOfWayRule identified by `id`.
  ///
  /// Throws an exception if `id` is unrecognized, which should be the
  /// case if no such rule exists or if the rule has only static semantics.
  // TODO(maddog@tri.global)  Better to throw exception or return an optional?
  RightOfWayRule::DynamicState GetState(const RightOfWayRule::Id& id) const {
    return DoGetState(id); }

 protected:
  RightOfWayStateProvider() = default;

 private:
  virtual RightOfWayRule::DynamicState DoGetState(
      const RightOfWayRule::Id& id) const = 0;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
