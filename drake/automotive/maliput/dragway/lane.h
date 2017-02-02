#pragma once

#include <memory>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"

namespace drake {
namespace maliput {
namespace dragway {

// Forward declarations.
class BranchPoint;
class Segment;

/// Dragway's implementation of api::Lane. It is an abstract parent class of
/// NorthBoundLane and SouthBoundLane. The lane is flat with a height of zero.
///
/// This class's constructor takes as input a parameter called `y_offset`. This
/// specifies the offset of this lane's frame's Y-axis relative to the world
/// frame's Y-axis.
///
/// Let Y_W be a Y-coordinate in this world's frame and Y_L be the
/// corresponding coordinate in the lane's frame. The relationship between
/// Y_W and Y_L is defined as follows.
///
///    Y_W = Y_L + y_offset
///
class Lane : public api::Lane {
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
  /// @param y_offset The conversion of a lane's y-axis coordinate into
  /// the world frame's y-axis coordinate.
  ///
  /// @param lane_bounds nominal bounds of the lane, uniform along the entire
  ///        reference path, which must be a subset of @p driveable_bounds.
  ///
  /// @param driveable_bounds The driveable bounds of the lane, uniform along
  ///        the entire reference path.
  ///
  Lane(const Segment* segment, const api::LaneId& id,  int index, double length,
      double y_offset, const api::RBounds& lane_bounds,
      const api::RBounds& driveable_bounds);

  virtual ~Lane()= default;

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

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const final;

  const Segment* segment_{};  // The segment to which this lane belongs.
  const api::LaneId id_;
  const int index_{};  // The index of this lane within a Segment.
  const double length_{};
  const double y_offset_{};
  const api::RBounds lane_bounds_;
  const api::RBounds driveable_bounds_;

  // BranchPoint* start_bp_{};
  // BranchPoint* end_bp_{};

  std::unique_ptr<api::BranchPoint> branch_point_;

  api::Lane* lane_to_left_{};
  api::Lane* lane_to_right_{};
};


}  // namespace dragway
}  // namespace maliput
}  // namespace drake
