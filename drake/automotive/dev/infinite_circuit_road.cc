#include "drake/automotive/dev/infinite_circuit_road.h"

#include <cmath>
#include <map>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace utility {


InfiniteCircuitRoad::InfiniteCircuitRoad(
    const api::RoadGeometryId& id,
    const api::RoadGeometry* source,
    const api::LaneEnd& start,
    const std::vector<const api::Lane*>& path)
    : id_(id),
      source_(source),
      junction_({id.id + ".junction"}, this),
      segment_({id.id + ".segment"}, this),
      lane_({id.id + ".lane"}, this, source, start, path) {}


InfiniteCircuitRoad::~InfiniteCircuitRoad() {}

const api::RoadGeometryId
InfiniteCircuitRoad::do_id() const { return id_; }

int
InfiniteCircuitRoad::do_num_junctions() const { return 1; }

const api::Junction*
InfiniteCircuitRoad::do_junction(int index) const { return &junction_; }

int
InfiniteCircuitRoad::do_num_branch_points() const { return 0; }

const api::BranchPoint*
InfiniteCircuitRoad::do_branch_point(int index) const { DRAKE_ABORT(); }

// TODO(maddog@tri.global) Implement when someone needs this.
api::RoadPosition
InfiniteCircuitRoad::DoToRoadPosition(
    const api::GeoPosition& geo_pos,
    const api::RoadPosition* hint,
    api::GeoPosition* nearest_point,
    double* distance) const { DRAKE_ABORT(); }

double
InfiniteCircuitRoad::do_linear_tolerance() const {
  return source_->linear_tolerance();
}

double
InfiniteCircuitRoad::do_angular_tolerance() const {
  return source_->angular_tolerance();
}


InfiniteCircuitRoad::Junction::Junction(const api::JunctionId& id,
                                        const InfiniteCircuitRoad* road)
    : id_(id), road_(road) {}

const api::JunctionId
InfiniteCircuitRoad::Junction::do_id() const { return id_; }

const api::RoadGeometry*
InfiniteCircuitRoad::Junction::do_road_geometry() const { return road_; }

int
InfiniteCircuitRoad::Junction::do_num_segments() const { return 1; }

const api::Segment*
InfiniteCircuitRoad::Junction::do_segment(int index) const {
  DRAKE_DEMAND(index == 0);
  return &road_->segment_;
}


InfiniteCircuitRoad::Segment::Segment(const api::SegmentId& id,
                                      const InfiniteCircuitRoad* road)
    : id_(id), road_(road) {}

const api::SegmentId
InfiniteCircuitRoad::Segment::do_id() const { return id_; }

const api::Junction*
InfiniteCircuitRoad::Segment::do_junction() const { return &road_->junction_; }

int
InfiniteCircuitRoad::Segment::do_num_lanes() const { return 1; }

const api::Lane*
InfiniteCircuitRoad::Segment::do_lane(int index) const {
  DRAKE_DEMAND(index == 0);
  return &road_->lane_;
}


InfiniteCircuitRoad::Lane::Lane(const api::LaneId& id,
                                const InfiniteCircuitRoad* road,
                                const api::RoadGeometry* source,
                                const api::LaneEnd& start,
                                const std::vector<const api::Lane*>& path)
    : id_(id),
      road_(road) {
  // Starting at start, walk source's Lane/BranchPoint graph.
  //
  // If path is non-empty, use that to guide us at each branch-point,
  // and stop when we get to the end of path.
  //
  // If path is empty, we find our own way by simply picking the first
  // ongoing branch at each branch-point.  We assume (demand!) that
  // there are no dead-end branch-points encountered along the way, so
  // we will eventually encounter a LaneEnd that we have seen before,
  // at which point we know that we have found a cycle.
  //
  // In either case, along the way, we will keep track of accumulated
  // s-length over the sequence of lanes.

  // Starting-ends of recorded lane traversals, mapped to index in sequence.
  std::map<api::LaneEnd, int, api::LaneEnd::StrictOrder> seen_records;
  double start_s = 0.;
  api::LaneEnd current = start;
  auto path_it = path.begin();

  while (true) {
    drake::log()->debug("walk lane {}  end {}    length {}",
                        current.lane->id().id,
                        current.end,
                        current.lane->length());
    const double end_s = start_s + current.lane->length();
    seen_records[current] = records_.size();
    record_map_.emplace(end_s, records_.size());
    records_.emplace_back(current.lane, start_s,
                          (current.end == api::LaneEnd::kFinish));
    start_s = end_s;

    // If a path was specified, and we've reached its end, then we are done.
    if ((path.size() > 0) && (path_it == path.end())) {
      break;
    }

    api::LaneEnd::Which other_end =
        (current.end == api::LaneEnd::kStart) ?
        api::LaneEnd::kFinish :
        api::LaneEnd::kStart;
    const api::LaneEndSet* branches =
        current.lane->GetOngoingBranches(other_end);
    DRAKE_DEMAND(branches->size() > 0);

    // If a non-empty path has been supplied, follow it.
    // Otherwise, simply use the first branch every time.
    if (path.size() > 0) {
      const api::Lane* specified_lane = *path_it;
      int bi = 0;
      while (branches->get(bi).lane != specified_lane) {
        ++bi;
        if (bi >= branches->size()) {
          drake::log()->error(
              "OOOPS:  Lane {} was not found at {} end of lane {}.",
              specified_lane->id().id,
              ((current.end == api::LaneEnd::kStart) ? "start" : "finish"),
              current.lane->id().id);
          DRAKE_ABORT();
        }
      }
      current = branches->get(bi);
      ++path_it;
    } else {
      current = branches->get(0);
    }
    drake::log()->debug("{} branches,  0 ---> lane {}, end {}",
                        branches->size(), current.lane->id().id, current.end);

    // If no path was specified and we are about to start at a lane-end that
    // we have already seen, then we are done.
    if ((path.size() == 0) && (seen_records.count(current))) {
      break;
    }
  }

  // If we forged our own path, and our last lane-end is not the same as our
  // first lane-end (i.e., its index is not zero), then we need to trim
  // records from the beginning (since they are not part of the circuit).
  if ((path.size() == 0) && (seen_records[current] > 0)) {
    records_.erase(records_.begin(),
                   records_.begin() + seen_records[current]);
    // Need to re-measure all the start/end offsets, too.
    record_map_.clear();
    start_s = 0;
    int index = 0;
    for (Record& r : records_) {
      r.start_circuit_s = start_s;
      start_s += r.lane->length();
      record_map_.emplace(start_s, index);
      ++index;
    }
  }

  cycle_length_ = start_s;
}


const api::LaneId
InfiniteCircuitRoad::Lane::do_id() const { return id_; }

const api::Segment*
InfiniteCircuitRoad::Lane::do_segment() const { return &road_->segment_; }

// Only one lane per segment!
int
InfiniteCircuitRoad::Lane::do_index() const { return 0; }

const api::Lane*
InfiniteCircuitRoad::Lane::do_to_left() const { return nullptr; }

const api::Lane*
InfiniteCircuitRoad::Lane::do_to_right() const { return nullptr; }

// An infinite lane has no branch-points....
const api::BranchPoint*
InfiniteCircuitRoad::Lane::DoGetBranchPoint(api::LaneEnd::Which) const {
  DRAKE_ABORT();
}

const api::LaneEndSet*
InfiniteCircuitRoad::Lane::DoGetConfluentBranches(api::LaneEnd::Which) const {
  DRAKE_ABORT();
}

const api::LaneEndSet*
InfiniteCircuitRoad::Lane::DoGetOngoingBranches(api::LaneEnd::Which) const {
  DRAKE_ABORT();
}

std::unique_ptr<api::LaneEnd>
InfiniteCircuitRoad::Lane::DoGetDefaultBranch(api::LaneEnd::Which) const {
  DRAKE_ABORT();
}


double InfiniteCircuitRoad::Lane::do_length() const { return INFINITY; }


api::RBounds
InfiniteCircuitRoad::Lane::do_lane_bounds(const double s) const {
  api::RoadPosition rp;
  bool is_reversed;
  std::tie(rp, is_reversed) = ProjectToSourceRoad({s, 0., 0.});
  const api::RBounds bounds = rp.lane->lane_bounds(rp.pos.s);
  return (is_reversed) ? api::RBounds(-bounds.r_max, -bounds.r_min) : bounds;
}


api::RBounds
InfiniteCircuitRoad::Lane::do_driveable_bounds(double s) const {
  api::RoadPosition rp;
  bool is_reversed;
  std::tie(rp, is_reversed) = ProjectToSourceRoad({s, 0., 0.});
  const api::RBounds bounds = rp.lane->driveable_bounds(rp.pos.s);
  return (is_reversed) ? api::RBounds(-bounds.r_max, -bounds.r_min) : bounds;
}


api::GeoPosition
InfiniteCircuitRoad::Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  const api::RoadPosition rp = ProjectToSourceRoad(lane_pos).first;
  return rp.lane->ToGeoPosition(rp.pos);
}


api::Rotation InfiniteCircuitRoad::Lane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  api::RoadPosition rp;
  bool is_reversed;
  std::tie(rp, is_reversed) = ProjectToSourceRoad(lane_pos);
  api::Rotation result = rp.lane->GetOrientation(rp.pos);
  if (is_reversed) {
    result.roll = -result.roll;
    result.pitch = -result.pitch;
    result.yaw = result.yaw + M_PI;
  }
  return result;
}


api::LanePosition InfiniteCircuitRoad::Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {
  api::RoadPosition rp;
  bool is_reversed;
  std::tie(rp, is_reversed) = ProjectToSourceRoad(position);
  api::LanePosition position_dot =
      rp.lane->EvalMotionDerivatives(
          rp.pos,
          is_reversed ? api::IsoLaneVelocity(-velocity.sigma_v,
                                             -velocity.rho_v,
                                             velocity.eta_v) : velocity);
  if (is_reversed) {
    position_dot.s *= -1;
    position_dot.r *= -1;
  }
  return position_dot;
}

api::GeoPosition InfiniteCircuitRoad::Lane::DoEvalGeoMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {
  DRAKE_ABORT();  // TODO(maddog@tri.global) Implement me.
}

int InfiniteCircuitRoad::Lane::GetPathIndex(const double s) const {
  double circuit_s = this->circuit_s(s);
  auto bound = record_map_.upper_bound(circuit_s);
  if (bound != record_map_.end()) {
    return bound->second;
  }
  drake::log()->error("UH OH {}   cycle {}   last {}",
                      circuit_s, cycle_length_,
                      records_.back().start_circuit_s +
                      records_.back().lane->length());
  DRAKE_ABORT();  // I.e., how did we fall off the end?
}


std::pair<api::RoadPosition, bool>
InfiniteCircuitRoad::Lane::ProjectToSourceRoad(
    const api::LanePosition& lane_pos) const {
  double circuit_s = this->circuit_s(lane_pos.s);
  const Record& r = records_[GetPathIndex(circuit_s)];
  const double s_offset = circuit_s - r.start_circuit_s;
  if (r.is_reversed) {
    // If this Lane is connected "backwards", then we have to measure s
    // from the end, and flip the sign of r.
    return std::make_pair(
        api::RoadPosition(
            r.lane, api::LanePosition(r.lane->length() - s_offset,
                                      -lane_pos.r,
                                      lane_pos.h)),
        true /*reversed*/);
  } else {
    return std::make_pair(
        api::RoadPosition(
            r.lane, api::LanePosition(s_offset, lane_pos.r, lane_pos.h)),
        false /*not reversed*/);
  }
}

}  // namespace utility
}  // namespace maliput
}  // namespace drake
