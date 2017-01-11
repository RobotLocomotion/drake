#pragma once

#include <vector>

#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

namespace drake {
namespace automotive {
namespace internal {

const double kEnormousDistance {1e12};

const double kCarLength {4.6};  // TODO(maddog) Get from somewhere else.

enum LaneRelation { kIntersection,
                    kMerge,
                    kSplit,
                    kSplitMerge,
                    kTangentLoops };

// State of a car in the underlying source maliput::api::RoadGeometry.
struct SourceState {
  SourceState() {}

  SourceState(maliput::api::RoadPosition arp, double als)
      : rp(arp), longitudinal_speed(als) {}

  maliput::api::RoadPosition rp;
  double longitudinal_speed{};
};


// Element of a car's travel path in the source maliput::api::RoadGeometry.
struct PathRecord {
  const maliput::api::Lane* lane{};
  bool is_reversed{};
};

void UnwrapEndlessRoadCarState(
    const std::vector<const EndlessRoadCarState<double>*>& car_inputs,
    const maliput::utility::InfiniteCircuitRoad* road,
    const double horizon_seconds,
    std::vector<SourceState>* source_states,
    std::vector<std::vector<PathRecord>>* paths);

void AssessLongitudinal(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs);

LaneRelation DetermineLaneRelation(const PathRecord& pra,
                                   const PathRecord& prb);

void AssessIntersections(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs);

}  // namespace internal
}  // namespace automotive
}  // namespace drake
