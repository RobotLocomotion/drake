#pragma once

#include <vector>

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {


/// Directed, inclusive longitudinal (s value) range from s0 to s1.
class DRAKE_DEPRECATED_AUTOMOTIVE
    SRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SRange);

  /// Default constructor:  initializes s0 and s1 to zero.
  SRange() = default;

  /// Constructs range [s0 --> s1].
  SRange(double s0, double s1) : s0_(s0), s1_(s1) {}

  /// Gets s0 value.
  double s0() const { return s0_; }

  /// Gets s1 value.
  double s1() const { return s1_; }

  /// Sets s0 value.
  void set_s0(double s0) { s0_ = s0; }

  /// Sets s1 value.
  void set_s1(double s1) { s1_ = s1; }

 private:
  double s0_{0.};
  double s1_{0.};
};


/// Directed longitudinal range of a specific Lane, identified by a LaneId.
class DRAKE_DEPRECATED_AUTOMOTIVE
    LaneSRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneSRange);

  /// Constructs a LaneSRange as `s_range` on Lane `lane_id`.
  LaneSRange(const LaneId& lane_id, const SRange& s_range)
      : lane_id_(lane_id), s_range_(s_range) {}

  /// Gets the LaneId.
  const LaneId& lane_id() const { return lane_id_; }

  /// Gets the SRange.
  SRange s_range() const { return s_range_; }

 private:
  LaneId lane_id_;
  SRange s_range_;
};


// TODO(maddog@tri.global) Figure out if there would be any loss or gain of
//                         utility if the contiguity requirement were removed.
/// A longitudinal route, possibly spanning multiple (end-to-end) lanes.
///
/// The sequence of LaneSRanges should be contiguous.  (In other words,
/// taken as a Lane-space path with r=0 and h=0, it should present a
/// G1-continuous curve.)
class DRAKE_DEPRECATED_AUTOMOTIVE
    LaneSRoute {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneSRoute);

  /// Default constructor:  constructs an empty route.
  LaneSRoute() = default;

  /// Constructs a LaneSRoute from the given sequence of LaneSRanges.
  explicit LaneSRoute(const std::vector<LaneSRange>& ranges)
      : ranges_(ranges) {}

  /// Returns the sequence of LaneSRanges.
  const std::vector<LaneSRange>& ranges() const { return ranges_; }

  // TODO(maddog@tri.global)  Implement a "CheckInvariants()" method which
  //                          ensures contiguity (with respect to a specified
  //                          RoadGeometry).

 private:
  std::vector<LaneSRange> ranges_;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
