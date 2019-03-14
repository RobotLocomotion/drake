#pragma once

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Rule describing direction usage.
///
/// Each rule instance describes the direction of travel for a longitudinal
/// portion of a Lane (which may be the entire length of the Lane).
///
/// Each instance is tagged with a severity.
///
/// More than one DirectionUsage rule should not be active for a given
/// portion of a lane.
class DirectionUsageRule {
 public:
  using Id = TypeSpecificIdentifier<class DirectionUsageRule>;

  /// Severity classification.
  enum class Severity {
    kStrict = 0,  /// No vehicle should travel on this lane in violation of this
                  /// rule.
    kPreferred    /// Vehicles should avoid travelling against this rule but
                  /// certain exceptions apply. E.g. passing is allowed.
  };

  /// Direction of travel classification. Categorized based on travel with or
  /// against the central axis (+S) of the lane frame.
  enum class Direction {
    kWithS = 0,  /// Travel should proceed in the direction of the +S axis.
    kAgainstS,   /// Travel should proceed opposite the +S axis direction.
    kBoth        /// Travel is allowed both with the lane direction (+S) or
                 /// against it.
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirectionUsageRule);

  /// Constructs a DirectionUsageRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param severity Severity of the rule
  /// @param direction the travel direction for the rule
  DirectionUsageRule(const Id& id, const LaneSRange& zone, Severity severity,
                     Direction direction)
      : id_(id), zone_(zone), severity_(severity), direction_(direction) {}

  /// Returns the persistent identifier.
  Id id() const { return id_; }

  /// Returns the zone to which this rule instance applies.
  const LaneSRange& zone() const { return zone_; }

  /// Returns the Severity of this rule instance.
  Severity severity() const { return severity_; }

  /// Returns the Direction of this rule instance.
  Direction direction() const { return direction_; }

 private:
  Id id_;
  LaneSRange zone_;
  Severity severity_{};
  Direction direction_{};
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
