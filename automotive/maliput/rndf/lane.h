#pragma once

#include <memory>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace rndf {

class BranchPoint;

/// Base class for the RNDF implementation of api::Lane.
///
/// This is the base class for subclasses, each of which describe a
/// primitive reference curve in the xy ground-plane of the world frame.
/// The specific curve is expressed by a subclass's implementations of
/// private virtual functions.
///
/// This base implementation will handle all the non-geometric stuff from the
/// lane. All geometric computation will be moved to each sub lane childs. See
/// SplineLane for an example.
class Lane : public api::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  /// Constructs a Lane.
  /// @param id is the ID of the api::Lane.
  /// @param segment is a pointer that refers to its parent, which must remain
  /// valid for the lifetime of this class.
  /// @param index is the index that can be used to reference this Lane from
  /// api::Segment::lane() call.
  Lane(const api::LaneId& id, const api::Segment* segment, int index)
      : id_(id), segment_(segment), index_(index) {}

  /// Sets the pointer of the BranchPoint that contains a
  /// api::LaneEnd::Which::kStart value attached to this lane pointer.
  /// @param bp should be a valid BranchPoint pointer.
  void SetStartBp(BranchPoint* bp) { start_bp_ = bp; }

  /// Sets the pointer of the BranchPoint that contains a
  /// api::LaneEnd::Which::kFinish value attached to this lane pointer.
  /// @param bp should be a valid BranchPoint pointer.
  void SetEndBp(BranchPoint* bp) { end_bp_ = bp; }

  /// Getter of the BranchPoint set as starting.
  /// @return The starting BranchPoint pointer.
  BranchPoint* start_bp() { return start_bp_; }

  /// Getter of the BranchPoint set as ending.
  /// @return The ending BranchPoint pointer.
  BranchPoint* end_bp() { return end_bp_; }

  ~Lane() override = default;

 private:
  const api::LaneId do_id() const override { return id_; }

  const api::Segment* do_segment() const override;

  int do_index() const override { return index_; }

  const api::Lane* do_to_left() const override {
    if ((segment_->num_lanes() - 1) == index_) {
      return nullptr;
    }
    return segment_->lane(index_ + 1);
  }

  const api::Lane* do_to_right() const override {
    if (index_ == 0) {
      return nullptr;
    }
    return segment_->lane(index_ - 1);
  }

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
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};
  const int index_{};
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
