#pragma once

#include <memory>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace maliput {
namespace dragway {

// Forward declarations.
class BranchPoint;
class Segment;

/**
 Dragway's implementation of api::Lane. The lane is flat with a height of
 zero.

 The following lane is implemented:

  <pre>
                     lane_bounds
        |<------------------------------->|
                 driveable_bounds
    |<--------------------------------------->|

    -------------------------------------------  ———  s = length()
    |                    :                    |   ^
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |           world
    |                    :                    |   |           frame:
    |                    :                    |   |
    |                    :                    |   |                X
    |                    :                    |   |                ^
    |                    :                    |   |                |
    |                    :                    |   v                |
    ---------------------o---------------------  ———  s = 0   Y <--o

            r_max                r_min
    |<-------------------|------------------->|

                                            y_offset
                         |<----------------------------------------|
  </pre>

  The lane's frame is defined by three coordinates: (`s`, `r`, `h`). Coordinate
  `s` is between zero and `length()`. It specifies the longitudinal traversal of
  the lane. Coordinate `r` is a value between `r_min` and `r_max`. It specifies
  the lateral traversal at a particular `s`. Coordinate `h` specifies the
  height above the lane's surface at a particular `s` and `r` (the lane's
  surface itself is always at `h = 0`). Since Dragway lanes are flat and level,
  `z = h` for all values of `s` and `r` and, in the Dragway's case, `z = 0` for
  the surface itself. The origin of the lane's frame is defined by the `o` along
  the above-shown `s = 0` line.

  Note: Each dragway lane has a teleportation feature at both ends: the
  (default) ongoing lane for LaneEnd::kFinish is LaneEnd::kStart of the same
  lane, and vice versa.
**/
class Lane final : public api::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  /// Constructs a dragway Lane.
  ///
  /// @param segment The Segment to which this lane belongs.
  ///
  /// @param id the lane ID. This can be any user-defined value.
  ///
  /// @param index The index of the lane with its Segment.
  ///
  /// @param length The total length of the lane.
  ///
  /// @param y_offset The vector from the world frame's origin to the
  /// lane's `s = 0` and `r = 0`. This value is positive when the lane's
  /// `s = 0` and `r = 0` is to the left of the world frame's origin, and is
  /// negative otherwise.
  ///
  /// @param lane_bounds nominal bounds of the lane, uniform along the entire
  ///        reference path, which must be a subset of @p driveable_bounds.
  ///
  /// @param driveable_bounds The driveable bounds of the lane, uniform along
  ///        the entire reference path.
  ///
  /// @param elevation_bounds The elevation bounds of the lane, uniform along
  ///        the entire reference path.
  ///
  Lane(const Segment* segment, const api::LaneId& id,  int index, double length,
       double y_offset, const api::RBounds& lane_bounds,
       const api::RBounds& driveable_bounds,
       const api::HBounds& elevation_bounds);

  ~Lane() final = default;

  /// @name Methods that set the lanes to the left and right of this lane.
  /// The corresponding accessors are api::Lane::to_left() and
  /// api::Lane::to_right().
  ///@{

  void set_lane_to_left(api::Lane* lane_to_left);
  void set_lane_to_right(api::Lane* lane_to_right);

  ///@}

  /// Returns the y-offset of this lane's frame relative to the world frame.
  double y_offset() const { return y_offset_; }

 private:
  const api::LaneId do_id() const final { return id_; }

  const api::Segment* do_segment() const final;

  int do_index() const final { return index_; }

  const api::Lane* do_to_left() const final { return lane_to_left_; }

  const api::Lane* do_to_right() const final { return lane_to_right_; }

  const api::BranchPoint* DoGetBranchPoint(
      const api::LaneEnd::Which which_end) const final;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd::Which which_end) const final;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd::Which which_end) const final;

  std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd::Which which_end) const final;

  double do_length() const final { return length_; }

  api::RBounds do_lane_bounds(double) const final;

  api::RBounds do_driveable_bounds(double) const final;

  api::HBounds do_elevation_bounds(double, double) const final;

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const final;

  api::GeoPosition DoToGeoPosition(const api::LanePosition& lane_pos) const
      final;

  api::Rotation DoGetOrientation(const api::LanePosition& lane_pos) const
      final;

  api::LanePosition DoToLanePosition(const api::GeoPosition& geo_pos,
                                     api::GeoPosition* nearest_point,
                                     double* distance) const final;

  api::LanePositionT<double> DoToLanePositionT(
      const api::GeoPositionT<double>& geo_pos,
      api::GeoPositionT<double>* nearest_point,
      double* distance) const final;

  api::LanePositionT<AutoDiffXd> DoToLanePositionT(
      const api::GeoPositionT<AutoDiffXd>& geo_pos,
      api::GeoPositionT<AutoDiffXd>* nearest_point,
      AutoDiffXd* distance) const final;

  template <typename T>
  api::LanePositionT<T> ImplDoToLanePositionT(
      const api::GeoPositionT<T>& geo_pos,
      api::GeoPositionT<T>* nearest_point,
      T* distance) const;

  const Segment* segment_{};  // The segment to which this lane belongs.
  const api::LaneId id_;
  const int index_{};  // The index of this lane within a Segment.
  const double length_{};
  const double y_offset_{};
  const api::RBounds lane_bounds_;
  const api::RBounds driveable_bounds_;
  const api::HBounds elevation_bounds_;

  // The following variable is actually `const` after construction.
  std::unique_ptr<api::BranchPoint> branch_point_;

  const api::Lane* lane_to_left_{};
  const api::Lane* lane_to_right_{};
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
