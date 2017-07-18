#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/drake_copyable.h"

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

/// Streams a string representation of @p lane_id into @p out. Returns @p out.
/// This method is provided for the purposes of debugging or text-logging. It is
/// not intended for serialization.
std::ostream& operator<<(std::ostream& out, const LaneId& lane_id);

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  virtual ~Lane() = default;

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

  /// Returns the elevation (`h`) bounds of the lane as a function of `(s, r)`.
  ///
  /// These are the elevation bounds for a position that is considered to be
  /// within the volume modelled by the RoadGeometry.
  HBounds elevation_bounds(double s, double r) const {
    return do_elevation_bounds(s, r); }

  /// Returns the GeoPosition corresponding to the given LanePosition.
  ///
  /// @pre The s component of @p lane_pos must be in domain [0, Lane::length()].
  /// @pre The r component of @p lane_pos must be in domain [Rmin, Rmax]
  ///      derived from Lane::driveable_bounds().
  GeoPosition ToGeoPosition(const LanePosition& lane_pos) const {
    return DoToGeoPosition(lane_pos);
  }

  /// Determines the LanePosition corresponding to GeoPosition @p geo_position.
  ///
  /// The return value is the LanePosition of the point within the Lane's
  /// driveable-bounds which is closest to @p geo_position (as measured by
  /// the Cartesian metric in the world frame).  If @p nearest_point is
  /// non-null, then it will be populated with the GeoPosition of that
  /// nearest point.  If @p distance is non-null, then it will be populated
  /// with the Cartesian distance from @p geo_position to that nearest point.
  ///
  /// This method guarantees that its result satisfies the condition that
  /// `ToGeoPosition(result)` is within `linear_tolerance()` of
  /// `*nearest_position`.
  LanePosition ToLanePosition(const GeoPosition& geo_position,
                              GeoPosition* nearest_point,
                              double* distance) const {
    return DoToLanePosition(geo_position, nearest_point, distance);
  }

  /// Generalization of ToLanePosition to arbitrary scalar types, where the
  /// structures `LanePositionT<T>` and `GeoPositionT<T>` are used in place of
  /// `LanePosition` and `GeoPosition`, respectively.
  ///
  /// Instantiated templates for the following kinds of T's are provided:
  /// - double
  /// - drake::AutoDiffXd
  ///
  /// They are already available to link against in the containing library.
  template <typename T>
  LanePositionT<T> ToLanePositionT(const GeoPositionT<T>& geo_position,
                                   GeoPositionT<T>* nearest_point,
                                   T* distance) const {
    return DoToLanePositionT(geo_position, nearest_point, distance);
  }

  // TODO(maddog@tri.global) Method to convert LanePosition to that of
  //                         another Lane.  (Should assert that both
  //                         lanes belong to same Segment.)  It should look
  //                         something like this:
  //           LanePosition ToOtherLane(const LanePosition& in_this_lane,
  //                                    const Lane* other_lane) const;

  /// Returns the rotation which expresses the orientation of the
  /// `Lane`-frame basis at @p lane_pos with respect to the
  /// world frame basis.
  Rotation GetOrientation(const LanePosition& lane_pos) const {
    return DoGetOrientation(lane_pos);
  }

  /// Computes derivatives of LanePosition given a velocity vector @p velocity.
  /// @p velocity is a isometric velocity vector oriented in the `Lane`-frame
  /// at @p position.
  ///
  /// @returns `Lane`-frame derivatives packed into a LanePosition struct.
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

 protected:
  Lane() = default;

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

  virtual HBounds do_elevation_bounds(double s, double r) const = 0;

  virtual GeoPosition DoToGeoPosition(const LanePosition& lane_pos) const = 0;

  virtual LanePosition DoToLanePosition(const GeoPosition& geo_position,
                                        GeoPosition* nearest_point,
                                        double* distance) const = 0;

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

  // Note that the `double` type must be in agreement with the explicit
  // instantiations for `ToLanePosition()` listed in lane.cc.
  virtual LanePositionT<double>
  DoToLanePositionT(const GeoPositionT<double>&,
                    GeoPositionT<double>*,
                    double*) const {
    DRAKE_ABORT();
  }

  // Note that the `AutoDiffXd` type must be in agreement with the explicit
  // instantiations for `ToLanePosition()` listed in lane.cc.
  virtual LanePositionT<AutoDiffXd>
  DoToLanePositionT(const GeoPositionT<AutoDiffXd>&,
                    GeoPositionT<AutoDiffXd>*,
                    AutoDiffXd*) const {
    DRAKE_ABORT();
  }
  ///@}
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
