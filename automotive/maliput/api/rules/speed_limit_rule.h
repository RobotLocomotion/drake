#pragma once

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Rule describing speed limits.
///
/// Each rule instance describes speed limits applied to a longitudinal
/// portion of a Lane (which may be the entire length of the Lane).  Each
/// instance is tagged with a severity; multiple instances with different
/// severities may apply to the same region of the road network.
///
/// Each instance mandates a maximum speed limit as well as a minimum
/// speed limit.  Since neither limit may be less than zero, a minimum
/// limit of exactly zero is equivalent to having no minimum limit at all.
class DRAKE_DEPRECATED_AUTOMOTIVE
    SpeedLimitRule {
 public:
  using Id = TypeSpecificIdentifier<class SpeedLimitRule>;

  /// Severity classification.
  enum class Severity {
    kStrict = 0,  ///< A strict limit is the established mandatory limit.
    kAdvisory     ///< An advisory limit is a recommendation, typically
                  ///  reflecting a road condition (e.g., a sharp curve).
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpeedLimitRule);

  /// Constructs a SpeedLimitRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param severity Severity of the rule
  /// @param min minimum speed
  /// @param max maximum speed
  ///
  /// `min` and `max` must be non-negative, and `min` must be less than
  /// or equal to `max`, otherwise a std::runtime_error is thrown.
  SpeedLimitRule(const Id& id, const LaneSRange& zone, Severity severity,
                 double min, double max)
      : id_(id), zone_(zone), severity_(severity), min_(min), max_(max) {
    DRAKE_THROW_UNLESS(min >= 0.);
    DRAKE_THROW_UNLESS(max >= 0.);
    DRAKE_THROW_UNLESS(min <= max);
  }

  /// Returns the persistent identifier.
  Id id() const { return id_; }

  /// Returns the zone to which this rule instance applies.
  const LaneSRange& zone() const { return zone_; }

  /// Returns the severity of this rule instance.
  Severity severity() const { return severity_; }

  /// Returns the minimum limit.
  double min() const { return min_; }

  /// Returns the maximum limit.
  double max() const { return max_; }

 private:
  Id id_;
  LaneSRange zone_;
  Severity severity_{};
  double min_{};
  double max_{};
  // TODO(maddog@tri.global)  Capture applicable vehicle types (e.g., 'trucks').
  // TODO(maddog@tri.global)  Capture time-of-day aspect (e.g., 'night only')
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
