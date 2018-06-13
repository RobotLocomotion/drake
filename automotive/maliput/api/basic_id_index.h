#pragma once

#include <unordered_map>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {


/// Basic general-purpose concrete implementation of the
/// RoadGeometry::IdIndex interface.
class BasicIdIndex : public RoadGeometry::IdIndex {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BasicIdIndex);

  BasicIdIndex() = default;
  ~BasicIdIndex() override = default;

  /// Adds @p lane to the index.
  ///
  /// @throws if @p lane's id() already exists in the index.
  /// @pre @p lane is not nullptr.
  void AddLane(const Lane* lane);

  /// Adds @p segment to the index.
  ///
  /// @throws if @p segment's id() already exists in the index.
  /// @pre @p segment is not nullptr.
  void AddSegment(const Segment* segment);

  /// Adds @p junction to the index.
  ///
  /// @throws if @p junction's id() already exists in the index.
  /// @pre @p junction is not nullptr.
  void AddJunction(const Junction* junction);

  /// Adds @p branch_point to the index.
  ///
  /// @throws if @p branch_point's id() already exists in the index.
  /// @pre @p branch_point is not nullptr.
  void AddBranchPoint(const BranchPoint* branch_point);

  /// Walks the object graph rooted at @p road_geometry and adds all
  /// components (Lane, Segment, Junction, BranchPoint) to the index.
  ///
  /// @throws if the graph of @p road_geometry contains any duplicate id's,
  ///         or if any of its id's already exist in the index.
  /// @pre @p road_geometry is not nullptr.
  void WalkAndAddAll(const RoadGeometry* road_geometry);

 private:
  const Lane* DoGetLane(const LaneId& id) const final;
  const Segment* DoGetSegment(const SegmentId& id) const final;
  const Junction* DoGetJunction(const JunctionId& id) const final;
  const BranchPoint* DoGetBranchPoint(const BranchPointId& id) const final;

  std::unordered_map<JunctionId, const Junction*> junction_map_;
  std::unordered_map<SegmentId, const Segment*> segment_map_;
  std::unordered_map<LaneId, const Lane*> lane_map_;
  std::unordered_map<BranchPointId, const BranchPoint*> branch_point_map_;
};


}  // namespace api
}  // namespace maliput
}  // namespace drake
