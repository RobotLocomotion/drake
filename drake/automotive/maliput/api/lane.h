#pragma once

#include "drake/automotive/maliput/api/lane_data.h"

#include <memory>
#include <string>

namespace drake {
namespace maliput {
namespace api {

class BranchPoint;
class Segment;
class LaneEndSet;


/// Persistent identifier for a Lane element.
struct LaneId {
  std::string id;
};


/// A Lane represents a lane of travel in a road network.  A Lane defines
/// a curvilinear coordinate system covering the road surface, with a
/// longitudinal 's' coordinate that expresses the arc-length along a
/// central reference curve.  The reference curve nominally represents
/// an ideal travel trajectory along the Lane.
///
/// Lanes are grouped by Segment.  All Lanes belonging to a Segment
/// represent the same road surface, but with different coordinate
/// parameterizations (e.g., each Lane has its own reference curve).
class Lane {
 public:
  virtual ~Lane() {}

  /// Returns the persistent identifier.
  const LaneId id() const { return do_id(); }

  /// Returns the Segment to which this Lane belongs.
  const Segment* segment() const { return do_segment(); }

  /// Returns the index of this Lane within the Segment which owns it.
  int index() const { return do_index(); }

  /// Returns a pointer to the adjacent Lane to the left of this Lane.
  ///
  /// Left is considered the +r direction with regards to the (s,r,h) frame,
  /// e.g., "to the left along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the left.
  const Lane* to_left() const { return do_to_left(); }

  /// Returns a pointer to the adjacent Lane to the right of this Lane.
  ///
  /// Right is considered the -r direction with regards to the (s,r,h) frame,
  /// e.g., "to the right along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the right.
  const Lane* to_right() const { return do_to_right(); }

  /// Returns the arc-length of the Lane along its reference curve.
  ///
  /// The value of length() is also the maximum s-coordinate for this Lane;
  /// i.e., the domain of s is [0, length()].
  double length() const { return do_length(); }

  /// Returns the nominal lateral (r) bounds for the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "staying in the lane".
  RBounds lane_bounds(double s) const { return do_lane_bounds(s); }

  /// Returns the driveable lateral (r) bounds of the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "on pavement", reflecting the physical extent of the paved surface of
  /// the lane's segment.
  RBounds driveable_bounds(double s) const { return do_driveable_bounds(s); }

  /// Returns the GeoPosition corresponding to the given LanePosition.
  ///
  /// @pre The s component of @p lane_pos must be in domain [0, Lane::length()].
  /// @pre The r component of @p lane_pos must be in domain [Rmin, Rmax]
  ///      derived from Lane::driveable_bounds().
  GeoPosition ToGeoPosition(const LanePosition& lane_pos) const {
    return DoToGeoPosition(lane_pos);
  }

  /// Calculates position in the LANE-space domain of the Lane which maps to
  /// the GEO-space point closest (per Cartesian metric) to the specified
  /// @p geo_pos.
  // TODO(maddog@tri.global)  UNDER CONSTRUCTION
  // TODO(maddog@tri.global)  Return the minimal-distance, too?
  // TODO(maddog@tri.global)  Select between lane_bounds and driveable_bounds?
  LanePosition ToLanePosition(const GeoPosition& geo_pos) const {
    return DoToLanePosition(geo_pos);
  }

  // TODO(maddog@tri.global) Method to convert LanePosition to that of
  //                         another Lane.  (Should assert that both
  //                         lanes belong to same Segment.)  It should look
  //                         something like this:
  //           LanePosition ToOtherLane(const LanePosition& in_this_lane,
  //                                    const Lane* other_lane) const;

  /// Return the rotation which expresses the orientation of the
  /// LANE-space basis at @p lane_pos with regards to the (single, global)
  /// GEO-space basis.
  Rotation GetOrientation(const LanePosition& lane_pos) const {
    return DoGetOrientation(lane_pos);
  }

  /// Compute derivatives of LanePosition given a velocity vector @p velocity.
  /// @p velocity is a isometric velocity vector oriented in the LANE-space
  /// reference frame at @p position.
  ///
  /// @returns LANE-space derivatives packed into a LanePosition struct.
  LanePosition EvalMotionDerivatives(const LanePosition& position,
                                     const IsoLaneVelocity& velocity) const {
    return DoEvalMotionDerivatives(position, velocity);
  }

  // TODO(maddog@tri.global)  Design/implement this.
  // void EvalSurfaceDerivatives(...) const { return do_(); }


  /// Returns the lane's BranchPoint for the end specified by @p which_end.
  const BranchPoint* GetBranchPoint(const LaneEnd::Which which_end) const {
    return DoGetBranchPoint(which_end);
  }

  /// Returns the set of LaneEnd's which connect with this lane on the
  /// same side of the BranchPoint at @p which_end.  At a minimum,
  /// this set will include this Lane.
  const LaneEndSet* GetConfluentBranches(
      const LaneEnd::Which which_end) const {
    return DoGetConfluentBranches(which_end);
  }

  /// Returns the set of LaneEnd's which continue onward from this lane at the
  /// BranchPoint at @p which_end.
  const LaneEndSet* GetOngoingBranches(
      const LaneEnd::Which which_end) const {
    return DoGetOngoingBranches(which_end);
  }

  /// Returns the default ongoing LaneEnd connected at @p which_end.
  ///
  /// @returns nullptr if no default branch has been established
  ///          at @p which_end.
  // TODO(maddog@tri.global)  The return type yearns to be
  //                          const boost::optional<LaneEnd>&.
  std::unique_ptr<LaneEnd> GetDefaultBranch(
      const LaneEnd::Which which_end) const {
    return DoGetDefaultBranch(which_end);
  }

  /// @name Deleted Copy/Move Operations
  /// Lane is neither copyable nor moveable.
  ///@{
  explicit Lane(const Lane&) = delete;
  Lane& operator=(const Lane&) = delete;
  ///@}

 protected:
  Lane() {}

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraints/invariants of the
  /// corresponding public methods.
  ///@{
  virtual const LaneId do_id() const = 0;

  virtual const Segment* do_segment() const = 0;

  virtual int do_index() const = 0;

  virtual const Lane* do_to_left() const = 0;

  virtual const Lane* do_to_right() const = 0;

  virtual double do_length() const = 0;

  virtual RBounds do_lane_bounds(double s) const = 0;

  virtual RBounds do_driveable_bounds(double s) const = 0;

  virtual GeoPosition DoToGeoPosition(const LanePosition& lane_pos) const = 0;

  virtual LanePosition DoToLanePosition(const GeoPosition& geo_pos) const = 0;

  virtual Rotation DoGetOrientation(const LanePosition& lane_pos) const = 0;

  virtual LanePosition DoEvalMotionDerivatives(
      const LanePosition& position, const IsoLaneVelocity& velocity) const = 0;

  virtual const BranchPoint* DoGetBranchPoint(
      const LaneEnd::Which which_end) const = 0;

  virtual const LaneEndSet* DoGetConfluentBranches(
      const LaneEnd::Which which_end) const = 0;

  virtual const LaneEndSet* DoGetOngoingBranches(
      const LaneEnd::Which which_end) const = 0;

  virtual std::unique_ptr<LaneEnd> DoGetDefaultBranch(
      const LaneEnd::Which which_end) const = 0;
  ///@}
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
