#pragma once

#include <map>
#include <vector>

#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

namespace drake {
namespace automotive {
namespace internal {

namespace api = maliput::api;
namespace utility = maliput::utility;

const double kEnormousDistance {1e12};

const double kEnormousVelocity {1e12};

// TODO(maddog) This should come from somewhere else, e.g., passed in.
// Based on the length of a 2010 Prius.
const double kCarLength {4.6};

enum LaneRelation { kIntersection,
                    kMerge,
                    kSplit,
                    kSplitMerge,
                    kTangentLoops };

// State of a car in the underlying source maliput::api::RoadGeometry.
struct SourceState {
  SourceState() {}

  SourceState(api::RoadPosition rp_in,
              double cos_heading_in,
              double circuit_s_speed_in)
      : rp(rp_in),
        cos_heading(cos_heading_in),
        circuit_s_speed(circuit_s_speed_in) {}
  /// Position in the source RoadGeometry.
  api::RoadPosition rp;
  /// Cosine of heading in (s,r) frame.
  double cos_heading;
  /// Rate of change of s-parameter along the infinite circuit.
  double circuit_s_speed{};
};

// Element of a car's travel path in the source maliput::api::RoadGeometry.
struct PathRecord {
  const api::Lane* lane{};
  bool is_reversed{};
};

// Record of when/where a car is expected to enter/exit a junction.
struct TimeBox {
  size_t car_index;
  PathRecord pr;
  double time_in;
  double time_out;
  double s_in;   // Distance to entry from current position.
  double s_out;  // Distance to exit from current position.
};

void UnwrapEndlessRoadCarState(
    const std::vector<const EndlessRoadCarState<double>*>& car_inputs,
    const utility::InfiniteCircuitRoad* road,
    const double horizon_seconds,
    std::vector<SourceState>* source_states,
    std::vector<std::vector<PathRecord>>* paths);

void AssessForwardPath(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs);

LaneRelation DetermineLaneRelation(const PathRecord& pra,
                                   const PathRecord& prb);

void AssessJunctions(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs);

void IndexJunctions(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    std::map<const api::Junction*, std::vector<TimeBox>>* boxes_by_junction,
    std::map<int, std::vector<const api::Junction*>>* junctions_by_car);

void MeasureJunctions(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::map<const api::Junction*,
    std::vector<TimeBox>>& boxes_by_junction,
    const std::map<int, std::vector<const api::Junction*>>& junctions_by_car,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs);


}  // namespace internal
}  // namespace automotive
}  // namespace drake
