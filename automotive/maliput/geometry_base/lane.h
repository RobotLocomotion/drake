#pragma once

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace geometry_base {

class BranchPoint;
class BranchPointBadge;
class SegmentBadge;

/// geometry_base's implementation of api::Lane.
class Lane : public api::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane);

  /// Constructs a Lane.
  ///
  /// @param id the ID of the Lane
  ///
  /// The Lane is not fully initialized until it is added to a Segment
  /// and each end is added to (one or two) BranchPoints.
  ///
  /// The Lane's index will be assigned when it is added to a Segment.
  explicit Lane(const api::LaneId& id) : id_(id) {}

  /// Returns a mutable pointer to the BranchPoint at the start end,
  /// or nullptr if the start end hasn't been added to a BranchPoint yet.
  BranchPoint* mutable_start_branch_point() { return start_branch_point_; }

  /// Returns a mutable pointer to the BranchPoint at the finish end,
  /// or nullptr if the finish end hasn't been added to a BranchPoint yet.
  BranchPoint* mutable_finish_branch_point() { return finish_branch_point_; }


#ifndef DRAKE_DOXYGEN_CXX
  // This may only be called, once, by a Segment.
  void AttachToSegment(const SegmentBadge&,
                       const api::Segment* segment, int index);

  // This may only be called, once, by a BranchPoint.
  void SetStartBranchPoint(const BranchPointBadge&, BranchPoint* branch_point);

  // This may only be called, once, by a BranchPoint.
  void SetFinishBranchPoint(const BranchPointBadge&, BranchPoint* branch_point);
#endif  // DRAKE_DOXYGEN_CXX

  ~Lane() override = default;

 private:
  const api::LaneId do_id() const override;

  const api::Segment* do_segment() const override;

  int do_index() const override;

  const api::Lane* do_to_left() const override;

  const api::Lane* do_to_right() const override;

  const api::BranchPoint* DoGetBranchPoint(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd::Which which_end) const override;

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneId id_;
  const api::Segment* segment_{};
  int index_{-1};
  BranchPoint* start_branch_point_{};
  BranchPoint* finish_branch_point_{};
};


}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
