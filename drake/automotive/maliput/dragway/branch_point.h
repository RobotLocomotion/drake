#pragma once

#include <map>
#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace dragway {

class BranchPoint;
class Lane;
class RoadGeometry;


/// Dragway's implementation of api::LaneEndSet. Since a dragway::Lane connects
/// to itself, this LaneEndSet only contains one api::LaneEnd.
class LaneEndSet : public api::LaneEndSet {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneEndSet)

  explicit LaneEndSet(const Lane* lane, api::LaneEnd::Which which_end) {
    ends_.emplace_back(lane, which_end);
  }

  ~LaneEndSet() override = default;

 private:
  int do_size() const override { return ends_.size(); }

  const api::LaneEnd& do_get(int index) const override { return ends_[index]; }

  std::vector<api::LaneEnd> ends_;
};


/// Dragway's implementation of api::BranchPoint.
class BranchPoint : public api::BranchPoint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BranchPoint)

  /// Constructs an empty BranchPoint.
  ///
  /// @param[in] id The ID of this branch point. It can be any user-specified
  /// value.
  ///
  /// @param[in] lane A pointer to the lane to which this branch point belongs.
  /// This pointer must remain valid for the lifetime of this class's instance.
  ///
  /// @param[in] road_geometry A pointer to the RoadGeometry to which this
  /// BranchPoint belongs. This pointer must remain valid for the lifetime of
  /// this class's instance.
  ///
  BranchPoint(const api::BranchPointId& id, const Lane* lane,
      const RoadGeometry* road_geometry);

  ~BranchPoint() override = default;

 private:
  const api::BranchPointId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd& end) const override;

  std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetASide() const override;

  const api::LaneEndSet* DoGetBSide() const override;

  api::BranchPointId id_;
  const RoadGeometry* road_geometry_{};
  LaneEndSet start_side_lane_end_set_;
  LaneEndSet finish_side_lane_end_set_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
