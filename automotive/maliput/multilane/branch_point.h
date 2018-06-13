#pragma once

#include <map>
#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace multilane {

class BranchPoint;
class Lane;


/// An implementation of LaneEndSet.
class LaneEndSet : public api::LaneEndSet {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneEndSet)

  LaneEndSet() = default;
  ~LaneEndSet() override = default;

  /// Adds a LaneEnd.
  void add(const api::LaneEnd& end) { ends_.push_back(end); }

 private:
  int do_size() const override { return ends_.size(); }

  const api::LaneEnd& do_get(int index) const override { return ends_[index]; }

  std::vector<api::LaneEnd> ends_;
};


/// An implementation of api::BranchPoint.
class BranchPoint : public api::BranchPoint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BranchPoint)

  /// Constructs an empty BranchPoint.
  ///
  /// @p road_geometry must remain valid for the lifetime of this class.
  BranchPoint(const api::BranchPointId& id,
              const api::RoadGeometry* road_geometry);

  /// Adds a LaneEnd to the "A side" of the BranchPoint.
  const api::LaneEnd& AddABranch(const api::LaneEnd& lane_end);

  /// Adds a LaneEnd to the "B side" of the BranchPoint.
  const api::LaneEnd& AddBBranch(const api::LaneEnd& lane_end);

  /// Sets the default branch for @p lane_end to @p default_branch.
  ///
  /// The specified LaneEnds must belong to opposite sides of this BranchPoint.
  void SetDefault(const api::LaneEnd& lane_end,
                  const api::LaneEnd& default_branch);

  ~BranchPoint() override = default;

 private:
  const api::BranchPointId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd& end) const override;

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetASide() const override { return &a_side_; }

  const api::LaneEndSet* DoGetBSide() const override { return &b_side_; }

  api::BranchPointId id_;
  const api::RoadGeometry* road_geometry_{};
  LaneEndSet a_side_;
  LaneEndSet b_side_;

  std::map<api::LaneEnd, LaneEndSet*,
           api::LaneEnd::StrictOrder> confluent_branches_;
  std::map<api::LaneEnd, LaneEndSet*,
           api::LaneEnd::StrictOrder> ongoing_branches_;
  std::map<api::LaneEnd, api::LaneEnd,
           api::LaneEnd::StrictOrder> defaults_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
