#pragma once

#include <map>
#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/geometry_base/lane_end_set.h"
#include "drake/automotive/maliput/geometry_base/passkey.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace geometry_base {

class BranchPoint;
class Lane;
class RoadGeometry;

/// geometry_base's implementation of api::BranchPoint.
class BranchPoint : public api::BranchPoint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BranchPoint);

  /// Constructs an empty BranchPoint.
  ///
  /// @param id the ID of the BranchPoint
  ///
  /// The BranchPoint is not fully initialized until it is added to a
  /// RoadGeometry.
  explicit BranchPoint(const api::BranchPointId& id);

  /// Adds the @p end of @p lane to the "A side" of this BranchPoint.
  ///
  /// @throws std::exception if `lane` is nullptr.
  void AddABranch(Lane* lane, api::LaneEnd::Which end) {
    AddBranch(lane, end, &a_side_);
  }

  /// Adds the @p end of @p lane to the "B side" of this BranchPoint.
  ///
  /// @throws std::exception if `lane` is nullptr.
  void AddBBranch(Lane* lane, api::LaneEnd::Which end) {
    AddBranch(lane, end, &b_side_);
  }

  /// Sets the default branch for @p lane_end to @p default_branch.
  ///
  /// @throws std::exception if the specified LaneEnds do not belong
  ///         to opposite sides of this BranchPoint.
  void SetDefault(const api::LaneEnd& lane_end,
                  const api::LaneEnd& default_branch);

  ~BranchPoint() override = default;


#ifndef DRAKE_DOXYGEN_CXX
  // Notifies BranchPoint of its parent RoadGeometry.
  // This may only be called, once, by a RoadGeometry.
  //
  // @param road_geometry  the parent RoadGeometry
  //
  // @pre `road_geometry` is non-null.
  // @pre Parent RoadGeometry has not already been set.
  void AttachToRoadGeometry(Passkey<RoadGeometry>,
                            const api::RoadGeometry* road_geometry);
#endif  // DRAKE_DOXYGEN_CXX

 private:
  // Common implementation for AddABranch() and AddBBranch().
  void AddBranch(Lane* lane, api::LaneEnd::Which end, LaneEndSet* side);

  api::BranchPointId do_id() const override;

  const api::RoadGeometry* do_road_geometry() const override;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd& end) const override;

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetASide() const override;

  const api::LaneEndSet* DoGetBSide() const override;

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


}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
