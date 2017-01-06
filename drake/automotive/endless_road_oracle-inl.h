#pragma once

/// @file
/// Template method implementations for endless_road_oracle.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "endless_road_oracle.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
EndlessRoadOracle<T>::EndlessRoadOracle(
    const maliput::utility::InfiniteCircuitRoad* road,
    const int num_cars)
    : road_(road), num_cars_(num_cars) {
  // Declare an input and output for each car.
  for (int i = 0; i < num_cars; ++i) {
    inports_.push_back(
        this->DeclareInputPort(systems::kVectorValued,
                               EndlessRoadCarStateIndices::kNumCoordinates,
                               systems::kContinuousSampling));
    outports_.push_back(
        this->DeclareOutputPort(systems::kVectorValued,
                                EndlessRoadOracleOutputIndices::kNumCoordinates,
                                systems::kContinuousSampling));
  }
}


template <typename T>
void EndlessRoadOracle<T>::EvalOutput(const systems::Context<T>& context,
                                      systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

//DBG  std::cerr << "EvalOutput()  ORACLE " << std::endl;

  // Obtain the inputs.
  std::vector<const EndlessRoadCarState<T>*> car_inputs;
  for (int i = 0; i < num_cars_; ++i) {
    const systems::VectorBase<T>* const vector_input =
        this->EvalVectorInput(context, i);
    DRAKE_ASSERT(vector_input);
    const EndlessRoadCarState<T>* const car_input =
        dynamic_cast<const EndlessRoadCarState<T>*>(vector_input);
    DRAKE_ASSERT(car_input);
    car_inputs.push_back(car_input);
  }

  // Obtain the output pointers.
  std::vector<EndlessRoadOracleOutput<T>*> oracle_outputs;
  for (int i = 0; i < num_cars_; ++i) {
    EndlessRoadOracleOutput<T>* const output_vector =
        dynamic_cast<EndlessRoadOracleOutput<T>*>(
            output->GetMutableVectorData(i));
    DRAKE_ASSERT(output_vector);
    oracle_outputs.push_back(output_vector);
  }

  DoEvalOutput(car_inputs, oracle_outputs);
}


namespace {

const double kEnormousDistance = 1e12;
const double kCarLength = 4.6;  // TODO(maddog) Get from somewhere else.

struct SourceState {
  SourceState() {}

  SourceState(maliput::api::RoadPosition arp, double als)
      : rp(arp), longitudinal_speed(als) {}

  maliput::api::RoadPosition rp;
  double longitudinal_speed{};
};


struct PathRecord {
  const maliput::api::Lane* lane{};
  bool is_reversed{};
};


void UnwrapEndlessRoadCarState(
    const std::vector<const EndlessRoadCarState<double>*>& car_inputs,
    const maliput::utility::InfiniteCircuitRoad* road,
    const double horizon_seconds,
    std::vector<SourceState>* source_states,
    std::vector<std::vector<PathRecord>>* paths) {

  source_states->clear();
  paths->clear();
  for (size_t i = 0; i < car_inputs.size(); ++i) {
    const EndlessRoadCarState<double>* self = car_inputs[i];

    const maliput::api::RoadPosition rp = road->ProjectToSourceRoad(
        {self->s(), 0., 0.}).first;
    // TODO(maddog)  Until we deal with cars going the wrong way.
    DRAKE_DEMAND(std::cos(self->heading()) >= 0.);
    DRAKE_DEMAND(self->speed() >= 0.);
    const double longitudinal_speed =
        self->speed() * std::cos(self->heading());
    source_states->emplace_back(rp, longitudinal_speed);

    const double horizon_meters = longitudinal_speed * horizon_seconds;
    // TODO(maddog)  Is this < constraint relevant anymore???
    DRAKE_DEMAND(horizon_meters < (0.5 * road->cycle_length()));
    DRAKE_DEMAND(horizon_meters >= 0.);
    const double circuit_s0 = road->lane()->circuit_s(self->s());

    paths->emplace_back();  // Add empty path-vector to paths vector.

    int path_index = road->GetPathIndex(circuit_s0);
    maliput::utility::InfiniteCircuitRoad::Record path_record =
        road->path_record(path_index);
    double circuit_s_in = circuit_s0;
    while (circuit_s_in <= (circuit_s0 + horizon_meters)) {
      (*paths)[i].push_back({path_record.lane, path_record.is_reversed});

      // TODO(maddog) Index should decrement for s_dot < 0.
      if (++path_index >= road->num_path_records()) {
        path_index = 0;
      }
      path_record = road->path_record(path_index);
      circuit_s_in = path_record.start_circuit_s;
      // Handle wrap-around of "circuit s" values.
      if (circuit_s_in < circuit_s0) { circuit_s_in += road->cycle_length(); }
    }
    DRAKE_DEMAND(!(*paths)[i].empty());
  }
}


void AssessLongitudinal(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs) {

  // Calculate longitudinal position of each car in the circuit.
  // Sort cars by longitudinal position:  use an ordered multimap that
  // maps s-position --> car-index.

  // Map{Lane* --> MultiMap{s --> car_index}}
  std::map<const maliput::api::Lane*,
           std::multimap<double, int>> cars_by_lane_and_s;

  for (size_t i = 0; i < source_states.size(); ++i) {
    const SourceState& self = source_states[i];
    cars_by_lane_and_s[self.rp.lane].emplace(self.rp.pos.s, i);
  }

  // For each car:
  //  - find nearest other car that is:
  //     a) ahead of self, and
  //     b) has a lateral position within XXXXXX bounds of self.
  //  - compute/record:
  //     * net true longitudinal distance to other car (delta sigma)
  //     * true longitudinal velocity difference (delta sigma-dot)
  // TODO(maddog)  Do something reasonable if num_cars_ < 2.
  for (size_t i = 0; i < source_states.size(); ++i) {
    const SourceState& self = source_states[i];
    const maliput::api::RoadPosition self_rp = self.rp;

//XXX    DRAKE_DEMAND(cars_by_lane_and_s[self_rp.lane][self_rp.pos.s] !=
//XXX                 cars_by_lane_and_s[self_rp.lane].end());

    double delta_position = kEnormousDistance;
    double delta_velocity = 0.;

    // Find the next car which is ahead of self, by at least a car-length.
    // Search along the sequence of lanes to be taken by self, starting with
    // the current lane.
    DRAKE_DEMAND(self_rp.lane == paths[i][0].lane);

    double sum = 0.;
    // TODO(maddog)  Review this; the notion is to skip self, and might as
    //               well skip any cars sooo close to self.
    double bound_nudge = 0.1 * kCarLength;
    bool is_first = true;
    auto path_it = paths[i].begin();
    while (path_it != paths[i].end()) {
      if (!path_it->is_reversed) {
        const double s0 = (is_first) ? self_rp.pos.s : 0.;
        const double lane_length = path_it->lane->length() - s0;
        DRAKE_DEMAND(lane_length > 0.);

        auto next_it = cars_by_lane_and_s[path_it->lane].upper_bound(
            s0 + bound_nudge);
        if (next_it != cars_by_lane_and_s[path_it->lane].end()) {
          const SourceState& next = source_states[next_it->second];
          const maliput::api::RoadPosition next_rp = next.rp;
          delta_position = sum + (next_rp.pos.s - s0);
          if (delta_position < 0.) {
            std::cerr << "FOR dp " << delta_position
                      << "   sum " << sum
                      << "   next-s " << next_rp.pos.s
                      << "   s0 " << s0
                      << "   first " << is_first
                      << std::endl;
          }
          // TODO(maddog)  Oy, this needs to account for travel direction
          //               of next in its source lane, not inf-circuit lane.
          delta_velocity = self.longitudinal_speed - next.longitudinal_speed;
          break;
        }
        sum += lane_length;
        bound_nudge = std::max(0., bound_nudge - lane_length);
      } else {
        /* Self is travelling this lane backwards. */
        const double s0 = (is_first) ? self_rp.pos.s : path_it->lane->length();
        const double lane_length = s0;
        DRAKE_DEMAND(lane_length > 0.);

        auto next_it = std::make_reverse_iterator(
            cars_by_lane_and_s[path_it->lane].lower_bound(s0 - bound_nudge));
        if (next_it != cars_by_lane_and_s[path_it->lane].rend()) {
          const SourceState& next = source_states[next_it->second];
          const maliput::api::RoadPosition next_rp = next.rp;
          delta_position = sum + (s0 - next_rp.pos.s);
          if (delta_position < 0.) {
            std::cerr << "REV dp " << delta_position
                      << "   sum " << sum
                      << "   next-s " << next_rp.pos.s
                      << "   s0 " << s0
                      << "   first " << is_first
                      << std::endl;
          }
          // TODO(maddog)  Oy, this needs to account for travel direction
          //               of next in its source lane, not inf-circuit lane.
          delta_velocity = self.longitudinal_speed - next.longitudinal_speed;
          break;
        }
        sum += lane_length;
        bound_nudge = std::max(0., bound_nudge - lane_length);
      }
      ++path_it;
      is_first = false;
    }

    // While within horizon...
    //   Traipse ahead to next branchpoint.
    //   Loop over all other-lanes on same side of BP as this-lane.
    //     Loop over all cars on lane
    //       If car is not heading toward BP, skip it.
    //       If car is closer to BP (farther from self, in parallel)
    //         than last best other, skip it.
    //       Mark down car as "best other" at X distance from BP.
    //     ...Recurse back into lanes leading to this lane, too.

    // TODO(maddog) Do a correct distance measurement (not just delta-s).
    const double net_delta_sigma = delta_position - kCarLength;
    if (net_delta_sigma <= 0.) {
      std::cerr << "TOO CLOSE!      delta_pos " << delta_position
                << "   car len " << kCarLength
                << "   nds " << net_delta_sigma
                << std::endl;
    }
    // If delta_position < kCarLength, the cars crashed!
    // TODO(maddog)  Or, they were passing, with lateral distance > width....
    DRAKE_DEMAND(net_delta_sigma > 0.);

    oracle_outputs[i]->set_net_delta_sigma(net_delta_sigma);
    oracle_outputs[i]->set_delta_sigma_dot(delta_velocity);
  }
}


enum LaneRelation { kIntersection, kMerge, kSplit };

LaneRelation DetermineLaneRelation(const PathRecord& pra,
                                   const PathRecord& prb) {
  // Assuming that the lanes described by pra and prb belong to the same
  // junction, determine how they related.

  // If they terminate at same BranchPoint, they merge.
  // Otherwise, if they originate at same BranchPoint, they split.
  // Otherwise, they merely intersect.
  // TODO(maddog)  Two other goofy cases:  split-merge, and tangent-loops.

  const maliput::api::BranchPoint* a_end_pt =
      pra.lane->GetBranchPoint(
          (pra.is_reversed) ? maliput::api::LaneEnd::kStart
          : maliput::api::LaneEnd::kFinish);
  const maliput::api::BranchPoint* b_end_pt =
      prb.lane->GetBranchPoint(
          (prb.is_reversed) ? maliput::api::LaneEnd::kStart
          : maliput::api::LaneEnd::kFinish);
  // TODO(maddog) Also ensure branches are confluent.
  if (b_end_pt == a_end_pt) { return kMerge; }

  const maliput::api::BranchPoint* a_start_pt =
      pra.lane->GetBranchPoint(
          (pra.is_reversed) ? maliput::api::LaneEnd::kFinish
          : maliput::api::LaneEnd::kStart);
  const maliput::api::BranchPoint* b_start_pt =
      prb.lane->GetBranchPoint(
          (prb.is_reversed) ? maliput::api::LaneEnd::kFinish
          : maliput::api::LaneEnd::kStart);
  // TODO(maddog) Also ensure branches are confluent.
  if (b_start_pt == a_start_pt) { return kSplit; }

  return kIntersection;
}


void AssessIntersections(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs) {

  // Indexing Phase.
  // For each car, measure/record the 'time-in' and 'time-out' to all junctions
  // expected to be traversed within kEventHorizon.
  struct TimeBox {
    size_t car_index;
    PathRecord pr;
    double time_in;
    double time_out;
    double s_in;  // Distance to entry from current position.
    double s_out; // Distance to exit from current position.
  };
  std::map<const maliput::api::Junction*,
           std::vector<TimeBox>> boxes_by_junction;
  std::map<int, std::vector<const maliput::api::Junction*>> junctions_by_car;

  for (size_t i = 0; i < source_states.size(); ++i) {
    const SourceState& self = source_states[i];
    DRAKE_DEMAND(self.rp.lane == paths[i][0].lane);

    double sum = 0.;
    bool is_first = true;
    for (const PathRecord& section: paths[i]) {
      double lane_length{};
      if (!section.is_reversed) {
        const double s0 = (is_first) ? self.rp.pos.s : 0.;
        lane_length = section.lane->length() - s0;
      } else {
        const double s0 = (is_first) ? self.rp.pos.s : section.lane->length();
        lane_length = s0;
      }
      DRAKE_DEMAND(lane_length > 0.);
      const double s_in = sum;
      sum += lane_length;
      const double s_out = sum;
      // TODO(maddog) Decide if better way to deal with zero speed.
      const double kNonZeroSpeed = 1e-12;
      const double speed = std::max(self.longitudinal_speed, kNonZeroSpeed);
      // TODO(maddog) time in/out should account for length of vehicle, too.
      const double time_in = s_in / speed;
      const double time_out = s_out / speed;
      const TimeBox box = TimeBox{i, section,
                                  time_in, time_out,
                                  s_in, s_out};

      const maliput::api::Junction* junction =
          section.lane->segment()->junction();
      boxes_by_junction[junction].push_back(box);
      junctions_by_car[i].push_back(junction);

      is_first = false;
    }
  }

  // Measure Phase.
  // For each car, find cars which are entering the same junction at the
  // same time as this car.  We only want to pay attention to cars coming
  // from the right, and ultimately only care about the nearest such car.
  for (size_t i = 0; i < source_states.size(); ++i) {

    double might_collide_at = kEnormousDistance;
    double maybe_diff_vel{};

    // Iterate over the junctions which car 'i' participates in.
    for (const auto& junction : junctions_by_car[i]) {

      // Find the TimeBox for this car.
      const TimeBox self_box = [&](){
        // TODO(maddog)  Linear search is dumb.
        for (const TimeBox& box : boxes_by_junction[junction]) {
          if (box.car_index == i) { return box; }
        }
        DRAKE_ABORT();
      }();
      DRAKE_DEMAND(self_box.time_in < self_box.time_out);

      const double kMergeHorizonCarLengths = 10.;
      const double kMergeHorizonSeconds = 2.;
      const double merge_horizon =
          (kMergeHorizonCarLengths * kCarLength) +
          (kMergeHorizonSeconds * source_states[i].longitudinal_speed);

      // If self is *already in* the intersection, then skip it.
      // (E.g., keep going and get out of the intersection!)
      if (self_box.time_in == 0.) { continue; }

      // Are there any intersecting TimeBoxes from other cars at this junction?
      for (const auto& other_box : boxes_by_junction[junction]) {
        // Skip self (car 'i').
        if (other_box.car_index == i) { continue; }
        // Skip same lane (which cannot be an intersecting path).
        if (other_box.pr.lane == self_box.pr.lane) { continue;}
        // TODO(maddog) It would probably be more efficient to keep track of
        //              "cars i,j merging at junction q" instead of rescanning
        //              here.
        // TODO(maddog) Isn't it the case, if two lanes in a junction
        //              have a branch-point in common, that they must be
        //              on the same a- or b-side of that branch-point?
        //              (I.e., they must be confluent branches of each other,
        //              not ongoing?)  This should be a checked invariant.

        switch (DetermineLaneRelation(self_box.pr, other_box.pr)) {
          case kIntersection: {
            // Skip other if it enters after self enters (i.e., self is first),
            // or exits before self enters (i.e., other is out of the way).
            DRAKE_DEMAND(other_box.time_in < other_box.time_out);
            if ((other_box.time_in > self_box.time_in) ||
                (other_box.time_out < self_box.time_in)) { continue; }

            // Finally:  there could possibly be a collision at this junction,
            // so consider self's position of entry as an obstacle.
            if (self_box.s_in < might_collide_at) {
              might_collide_at = self_box.s_in;
              maybe_diff_vel = source_states[i].longitudinal_speed - 0.;
            }
            break;
          }
          case kMerge: {
            // TODO(maddog)  Limit the time/distance horizon over which
            //               merges are considered --- horizon should be
            //               shorter than same-lane lookahead.
            if (self_box.s_out > merge_horizon) { continue; }

            const double delta_position = self_box.s_out - other_box.s_out;
            if (delta_position < 0.) {
              // Other is 'behind' self, so skip it.
              continue;
            }
            double net_delta_sigma = delta_position - kCarLength;
            if (net_delta_sigma <= 0.) {
              const double kTinyDistance = 0.01;
              net_delta_sigma = kTinyDistance;
            }
            if (net_delta_sigma < might_collide_at) {
              might_collide_at = net_delta_sigma;
              maybe_diff_vel =
                  source_states[i].longitudinal_speed -
                  source_states[other_box.car_index].longitudinal_speed;
            }
            break;
          }
          case kSplit: {
            // TODO(maddog) Handle split.
            continue;
          }
          default: { DRAKE_ABORT(); }
        }

        // TODO(maddog) Compare speeds?  We really want the slower car
        //              to go slower, and faster faster, right?
        // TODO(maddog) Skip cars coming from left.  (necessary?)
      }
    }

    if (might_collide_at < oracle_outputs[i]->net_delta_sigma()) {
      oracle_outputs[i]->set_net_delta_sigma(might_collide_at);
      oracle_outputs[i]->set_delta_sigma_dot(maybe_diff_vel);
    }
  }
}


}



template <typename T>
void EndlessRoadOracle<T>::DoEvalOutput(
    const std::vector<const EndlessRoadCarState<T>*>& car_inputs,
    std::vector<EndlessRoadOracleOutput<T>*>& oracle_outputs) const {
  // The goal here is, for each car, to calculate the distance and
  // differential velocity to the nearest other car/obstacle ahead.
  // We consider a couple of different varieties of "car/obstacle ahead":
  //  a) cars ahead on the InfiniteCircuitRoad circuit, which covers
  //      normal car-following behavior;
  //  b) cars approaching from the right, on intersecting lanes in
  //      the base RoadGeometry --- this hopefully produces some non-crashing
  //      behavior at intersections;
  // TODO(maddog)  Actually do part (c).
  //  c) cars in a merging lane to the left --- this hopefully produces
  //      so non-crashing merging behaviors.


  // Time to look-ahead for intersections.
  // TODO(maddog) Should be a multiple of IDM headway h.
  // TODO(maddog) Should have a distance component, too, as a multiple of
  //              car lengths.
  //////////  const T kEventHorizon{2.0};  // seconds
  const T kEventHorizon{10.0};  // seconds

  //
  // We really only want the InfiniteCircuitRoad to serve two functions:
  //  1)  until proper hybrid system support is available, emulate a
  //      uni-modal continuous system for the vehicle control;
  //  2)  represent the path of the cars through the road network.
  // Here we extract the RoadPositions of each car in the underlying
  // source RoadGeometry, and we extract their paths.
  std::vector<SourceState> source_states;
  std::vector<std::vector<PathRecord>> paths;
  UnwrapEndlessRoadCarState(car_inputs, road_, kEventHorizon,
                            &source_states, &paths);

  AssessLongitudinal(source_states, paths, oracle_outputs);
  AssessIntersections(source_states, paths, oracle_outputs);
}


template <typename T>
std::unique_ptr<systems::BasicVector<T>
                > EndlessRoadOracle<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadOracleOutput<T>>();
}

}  // namespace automotive
}  // namespace drake
