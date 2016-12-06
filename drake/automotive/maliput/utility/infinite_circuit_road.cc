#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/common/drake_assert.h"

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
      junction_({id.id + ".junction"}, this, &segment_),
      segment_({id.id + ".segment"}, &junction_, &lane_),
      lane_({id.id + ".lane"}, &segment_, source, start, path) {}


InfiniteCircuitRoad::~InfiniteCircuitRoad() {}



InfiniteCircuitRoad::Lane::Lane(const api::LaneId& id,
                                const Segment* segment,
                                const api::RoadGeometry* source,
                                const api::LaneEnd& start,
                                const std::vector<const api::Lane*>& path)
    : id_(id),
      segment_(segment) {
  // Starting at start, walk source's Lane/BranchPoint graph.  We
  // assume (demand!) that there are no dead-end branch-points, so we
  // will eventually encounter a LaneEnd that we have seen before, at
  // which point we know that we have found a cycle.  Along the way,
  // we will keep track of accumulated s-length over the sequence of
  // lanes.

  // Starting-ends of recorded lane traversals, mapped to index in sequence.
  std::map<api::LaneEnd, int, api::LaneEnd::StrictOrder> seen_records;
  double start_s = 0.;
  api::LaneEnd current = start;
  auto path_it = path.begin();

  while (true) {
    if ((path.size() == 0) && (seen_records.count(current))) {
      break;
    } else if ((path.size() > 0) && (path_it == path.end())) {
      break;
    }

    std::cerr << "walk lane " << current.lane->id().id
              << "  end " << current.end
              << "   length " << current.lane->length() << std::endl;
    const double end_s = start_s + current.lane->length();
    seen_records[current] = records_.size();
    records_.push_back(Record {
        current.lane, start_s, end_s, (current.end == api::LaneEnd::kFinish)});

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
          std::cerr << "OOOPS:  Lane " << specified_lane->id().id
                    << " was not found at "
                    << ((current.end == api::LaneEnd::kStart)
                        ? "start" : "finish")
                    << " end of lane " << current.lane->id().id << std::endl;
          DRAKE_ABORT();
        }
      }
      current = branches->get(bi);
      ++path_it;
    } else {
      current = branches->get(0);
    }
    std::cerr << branches->size() << " branches, "
              << " 0 ---> lane " << current.lane->id().id
              << ", end " << current.end
              << std::endl;

    start_s = end_s;
  }

  if ((path.size() == 0) && (seen_records[current] > 0)) {
    // We're not back at the start; need to trim records from the beginning.
    records_.erase(records_.begin(),
                   records_.begin() + seen_records[current]);
    // Need to re-measure all the start/end offsets, too.
    start_s = 0;
    for (Record& r : records_) {
      r.start_circuit_s = start_s;
      start_s += r.lane->length();
      r.end_circuit_s = start_s;
    }
  }

  cycle_length_ = start_s;
}


InfiniteCircuitRoad::Lane::~Lane() {}


double InfiniteCircuitRoad::Lane::do_length() const {
  return INFINITY;
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


int InfiniteCircuitRoad::Lane::GetPathIndex(const double s) const {
  double circuit_s = this->circuit_s(s);
  for (size_t i = 0; i < records_.size(); ++i) {
    const Record& r = records_[i];
    if (circuit_s < r.end_circuit_s) {
      return i;
    }
  }
  DRAKE_ABORT(); // I.e., how did we fall off the end?
}


std::pair<api::RoadPosition, bool>
InfiniteCircuitRoad::Lane::ProjectToSourceRoad(
    const api::LanePosition& lane_pos) const {
  double circuit_s = this->circuit_s(lane_pos.s);
  for (const Record& r : records_) {
    if (circuit_s < r.end_circuit_s) {
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
  }
  std::cerr << "UH OH " << circuit_s
            << "   cycle " << cycle_length_
            << "   last " << records_.back().end_circuit_s
            << std::endl;
  DRAKE_ABORT(); // I.e., how did we fall off the end?
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
