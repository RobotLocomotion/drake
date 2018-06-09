#include "drake/automotive/maliput/api/basic_id_index.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {

void BasicIdIndex::AddLane(const Lane* lane) {
  DRAKE_THROW_UNLESS(lane_map_.emplace(lane->id(), lane).second);
}


void BasicIdIndex::AddSegment(const Segment* segment) {
  DRAKE_THROW_UNLESS(segment_map_.emplace(segment->id(), segment).second);
}


void BasicIdIndex::AddJunction(const Junction* junction) {
  DRAKE_THROW_UNLESS(junction_map_.emplace(junction->id(), junction).second);
}


void BasicIdIndex::AddBranchPoint(const BranchPoint* branch_point) {
  DRAKE_THROW_UNLESS(
      branch_point_map_.emplace(branch_point->id(), branch_point).second);
}


void BasicIdIndex::WalkAndAddAll(const RoadGeometry* road_geometry) {
  for (int ji = 0; ji < road_geometry->num_junctions(); ++ji) {
    const Junction* junction = road_geometry->junction(ji);
    AddJunction(junction);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const Segment* segment = junction->segment(si);
      AddSegment(segment);
      for (int li = 0; li < segment->num_lanes(); ++li) {
        AddLane(segment->lane(li));
      }
    }
  }
  for (int bi = 0; bi < road_geometry->num_branch_points(); ++bi) {
    AddBranchPoint(road_geometry->branch_point(bi));
  }
}


namespace {
template <typename T, typename U>
T find_or_nullptr(const std::unordered_map<U, T>& map, const U& id) {
  auto it = map.find(id);
  return (it == map.end()) ? nullptr : it->second;
}
}  // namespace


const Lane* BasicIdIndex::DoGetLane(const LaneId& id) const {
  return find_or_nullptr(lane_map_, id);
}


const Segment* BasicIdIndex::DoGetSegment(const SegmentId& id) const {
  return find_or_nullptr(segment_map_, id);
}


const Junction* BasicIdIndex::DoGetJunction(const JunctionId& id) const {
  return find_or_nullptr(junction_map_, id);
}


const BranchPoint*
BasicIdIndex::DoGetBranchPoint(const BranchPointId& id) const {
  return find_or_nullptr(branch_point_map_, id);
}


}  // namespace api
}  // namespace maliput
}  // namespace drake
