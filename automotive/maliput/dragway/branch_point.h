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
namespace dragway {

class BranchPoint;
class Lane;

/// Dragway's implementation of api::LaneEndSet. Since a dragway::Lane connects
/// to itself, this LaneEndSet only contains one api::LaneEnd.
class LaneEndSet final : public api::LaneEndSet {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneEndSet)

  explicit LaneEndSet(const api::Lane* lane, api::LaneEnd::Which which_end)
      : end_(lane, which_end) {}

  ~LaneEndSet() override = default;

 private:
  int do_size() const override { return 1; }

  const api::LaneEnd& do_get(int) const override { return end_; }

  const api::LaneEnd end_;
};


/// Dragway's implementation of api::BranchPoint.
class BranchPoint final : public api::BranchPoint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BranchPoint)

  /// Constructs a fully initialized BranchPoint for a Dragway lane.
  ///
  /// @param[in] id The ID of this branch point. It can be any user-specified
  /// value.
  ///
  /// @param[in] lane A pointer to the lane to which this branch point belongs.
  /// This pointer must remain valid for the lifetime of this class's instance.
  ///
  /// @param[in] road_geometry A pointer to the %RoadGeometry to which this
  /// BranchPoint belongs. This pointer must remain valid for the lifetime of
  /// this class's instance.
  ///
  BranchPoint(const api::BranchPointId& id, const Lane* lane,
      const api::RoadGeometry* road_geometry);

  ~BranchPoint() final = default;

 private:
  const api::BranchPointId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd& end) const override;

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetASide() const override;

  const api::LaneEndSet* DoGetBSide() const override;

  const api::BranchPointId id_;
  const api::RoadGeometry* road_geometry_{};
  const LaneEndSet start_side_lane_end_set_;
  const LaneEndSet finish_side_lane_end_set_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
