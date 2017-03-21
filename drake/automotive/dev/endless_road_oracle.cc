/* clang-format off */
#include "drake/automotive/dev/endless_road_oracle.h"
#include "drake/automotive/dev/endless_road_oracle-internal.h"
/* clang-format on */

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/dev/infinite_circuit_road.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
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
    this->DeclareInputPort(systems::kVectorValued,
                           EndlessRoadCarStateIndices::kNumCoordinates);
    this->DeclareOutputPort(systems::kVectorValued,
                            EndlessRoadOracleOutputIndices::kNumCoordinates);
  }
}


template <typename T>
void EndlessRoadOracle<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
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

  ImplCalcOutput(car_inputs, oracle_outputs);
}



template <typename T>
void EndlessRoadOracle<T>::ImplCalcOutput(
    const std::vector<const EndlessRoadCarState<T>*>& car_inputs,
    const std::vector<EndlessRoadOracleOutput<T>*>& oracle_outputs) const {
  // The goal here is, for each car, to calculate the distance and
  // differential velocity to the nearest other car/obstacle ahead.
  // We consider a couple of different varieties of "car/obstacle ahead":
  //  a) cars ahead on the InfiniteCircuitRoad circuit, which covers
  //      normal car-following behavior;
  //  b) cars approaching on intersecting lanes in the base RoadGeometry
  //      (an anticipated collision makes the intersection be treated as
  //      a stationary obstacle);
  //  c) cars in a merging lane in the base RoadGeometry.

  // TODO(maddog@tri.global)  A more principled approach would probably make
  //                          this a multiple of the IDM headway parameter h,
  //                          but this value works well in the demo.  (Too
  //                          short and there will be collisions!)
  // Time to look-ahead for intersections.
  const T kEventHorizon{10.0};  // seconds

  // The InfiniteCircuitRoad serves two functions:
  //  1) emulate a uni-modal continuous system for the vehicle control until
  //     proper hybrid system support is available;
  //  2) represent the path of the cars through the source road network.
  //
  // Here we extract the RoadPositions of each car in the underlying
  // source RoadGeometry, and we extract their paths forward (within the
  // given horizon).
  std::vector<internal::SourceState> source_states;
  std::vector<std::vector<internal::PathRecord>> paths;
  UnwrapEndlessRoadCarState(car_inputs, road_, kEventHorizon,
                            &source_states, &paths);

  // Deal with goal (a).
  AssessForwardPath(source_states, paths, oracle_outputs);
  // Deal with goal (b) and goal (c).
  AssessJunctions(source_states, paths, oracle_outputs);
}


template <typename T>
std::unique_ptr<systems::BasicVector<T>
                > EndlessRoadOracle<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadOracleOutput<T>>();
}


namespace internal {

// EndlessRoadCarState is defined with respect to an InfiniteCircuitRoad,
// which is a wrapper of sorts around an underlying source RoadGeometry.
// Here, we unwrap the state of each car:  we map its state into the
// source RoadGeometry, and figure out its path in that road network
// a little ways into the future.
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

    const maliput::api::RoadPosition rp =
        road->lane()->ProjectToSourceRoad({self->s(), self->r(), 0.}).first;
    const double cos_heading = std::cos(self->heading());
    // TODO(maddog)  Until we deal with cars going the wrong way.
    DRAKE_DEMAND(cos_heading >= 0.);
    // TODO(maddog@tri.global)  Until we deal with cars going in reverse.
    DRAKE_DEMAND(self->speed() >= 0.);
    const double circuit_s_speed = self->speed() * cos_heading;
    // Save self's state projected into the source road-network.
    source_states->emplace_back(rp, cos_heading, circuit_s_speed);

    // (No point in making the horizon longer than one lap around the circuit.)
    const double horizon_meters = std::min(
        circuit_s_speed * horizon_seconds,
        road->lane()->cycle_length());
    DRAKE_DEMAND(horizon_meters >= 0.);

    const double circuit_s_now = road->lane()->circuit_s(self->s());
    const double circuit_s_horizon = circuit_s_now + horizon_meters;

    // Add an empty path-vector to paths vector for self.
    paths->emplace_back();
    std::vector<PathRecord>* self_path = &(paths->back());

    int path_index = road->lane()->GetPathIndex(circuit_s_now);
    maliput::utility::InfiniteCircuitRoad::Record path_record =
        road->lane()->path_record(path_index);
    double circuit_s_in = circuit_s_now;
    while (circuit_s_in <= circuit_s_horizon) {
      self_path->push_back(
          PathRecord {path_record.lane, path_record.is_reversed});

      // TODO(maddog) Index should decrement for speed < 0.
      if (++path_index >= road->lane()->num_path_records()) {
        path_index = 0;
      }
      path_record = road->lane()->path_record(path_index);
      circuit_s_in = path_record.start_circuit_s;
      // Handle wrap-around of "circuit s" values.
      if (circuit_s_in < circuit_s_now) {
        circuit_s_in += road->lane()->cycle_length();
      }
    }
    DRAKE_DEMAND(!self_path->empty());
  }
}


void AssessForwardPath(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs) {

  // Calculate longitudinal position of each car in the circuit.
  // Sort cars by longitudinal position:  use an ordered multimap that
  // maps s-position --> car-index.

  // Map of multimap:  Lane* --> s --> (car_index, ...)
  std::map<const maliput::api::Lane*,
           std::multimap<double, int>> cars_by_lane_and_s;

  for (size_t car_index = 0; car_index < source_states.size(); ++car_index) {
    const SourceState& self = source_states[car_index];
    cars_by_lane_and_s[self.rp.lane].emplace(self.rp.pos.s, car_index);
  }

  // For each car:
  //  - find nearest other car that is:
  //     a) ahead of self
  // TODO(maddog@tri.global)  AND b) has a lateral position within
  //                                 some X margin of self.
  //  - compute/record:
  //     * net longitudinal distance to other car (delta s)
  //     * longitudinal velocity difference (delta s-dot)
  for (size_t car_index = 0; car_index < source_states.size(); ++car_index) {
    const SourceState& self = source_states[car_index];
    const std::vector<PathRecord>& self_path = paths[car_index];

    double delta_position = kEnormousDistance;
    double delta_velocity = 0.;

    // Find the next car which is ahead of self, by at least a car-length.
    // Search along the sequence of lanes to be taken by self, starting with
    // the current lane.
    // TODO(maddog@tri.global)  To generalize over vehicles of different
    //   lengths, what we really want here is the 'SourceState.rp' to reflect
    //   the position of the *rear bumper*.

    // Sanity check:  current lane should be first lane on path.
    DRAKE_DEMAND(self.rp.lane == self_path[0].lane);

    double sum = 0.;
    auto path_it = self_path.begin();
    DRAKE_DEMAND(path_it != self_path.end());
    // Start looking at least one car-length ahead (e.g., ignore cars that we
    // have already collided with).
    const double skip_margin = kCarLength;
    double skip_length = path_it->is_reversed ?
        (path_it->lane->length() - self.rp.pos.s + skip_margin) :
        (self.rp.pos.s + skip_margin);
    for (; path_it != self_path.end(); ++path_it) {
      double s0{};
      double lane_length{};
      if (!path_it->is_reversed) {
        s0 = skip_length;
        lane_length = path_it->lane->length() - s0;
      } else {
        // Self is travelling this lane backwards.
        s0 = path_it->lane->length() - skip_length;
        lane_length = s0;
      }
      if (lane_length <= 0.) {
        skip_length = -lane_length;
        continue;
      }
      skip_length = 0.;

      const std::multimap<double, int>& cars_in_lane =
          cars_by_lane_and_s[path_it->lane];

      if (!path_it->is_reversed) {
        auto next_it = cars_in_lane.upper_bound(s0);
        if (next_it != cars_in_lane.end()) {
          const size_t next_car_index = next_it->second;
          const SourceState& next = source_states[next_car_index];
          delta_position = sum + (next.rp.pos.s - s0);
          if (delta_position < 0.) {
            drake::log()->warn(
                "FORWARD dp {}  sum {}  next-s {}  s0 {}  skip {}",
                delta_position, sum, next.rp.pos.s, s0, skip_length);
          }
          const double self_local_s_speed = self.circuit_s_speed;
          const double next_local_s_speed =
              next.circuit_s_speed *
              (paths[next_car_index][0].is_reversed ? -1. : 1.);
          delta_velocity = self_local_s_speed - next_local_s_speed;
          break;
        }
      } else {
        // TODO(maddog)  In the modern world, we can use the convenient
        //               std::make_reverse_iterator.
        auto next_it = std::reverse_iterator<
          std::multimap<double, int>::const_iterator>(
              cars_in_lane.lower_bound(s0));
        if (next_it != cars_in_lane.rend()) {
          const size_t next_car_index = next_it->second;
          const SourceState& next = source_states[next_car_index];
          delta_position = sum + (s0 - next.rp.pos.s);
          if (delta_position < 0.) {
            drake::log()->warn(
                "REVERSED dp {}  sum {}  next-s {}  s0 {}  skip {}",
                delta_position, sum, next.rp.pos.s, s0, skip_length);
          }
          const double self_local_s_speed = -self.circuit_s_speed;
          const double next_local_s_speed =
              next.circuit_s_speed *
              (paths[next_car_index][0].is_reversed ? -1. : 1.);
          delta_velocity = -(self_local_s_speed - next_local_s_speed);
          break;
        }
      }
      sum += lane_length;
    }
    // TODO(maddog@tri.global)  We are actually only calculating 'delta s'
    //                          here, not 'delta sigma'; consider doing it
    //                          the right way.
    const double net_delta_sigma = delta_position + skip_margin - kCarLength;
    if (net_delta_sigma <= 0.) {
      // TODO(maddog@tri.global)  Don't consider it a collision if the lateral
      //                          distance exceeds some safe width.
      drake::log()->error(
          "COLLISION!  delta_pos {}  car-length {}  net-delta-sigma {}",
          delta_position, kCarLength, net_delta_sigma);
    }
    // Transform delta_velocity back into forward-facing frame of the car.
    // TODO(maddog@tri.global)  We are actually only calculating 's-dot',
    //                          not 'sigma-dot'; consider doing it the
    //                          right way.
    const double delta_sigma_dot =
        (self.cos_heading == 0.) ?
        std::copysign(kEnormousVelocity, delta_velocity) :
        (delta_velocity / self.cos_heading);

    oracle_outputs[car_index]->set_net_delta_sigma(net_delta_sigma);
    oracle_outputs[car_index]->set_delta_sigma_dot(delta_sigma_dot);
  }
}


const maliput::api::LaneEnd GetStartEnd(const PathRecord& pr) {
  return maliput::api::LaneEnd(pr.lane,
                               (pr.is_reversed) ?
                               maliput::api::LaneEnd::kFinish :
                               maliput::api::LaneEnd::kStart);
}

const maliput::api::LaneEnd GetFinishEnd(const PathRecord& pr) {
  return maliput::api::LaneEnd(pr.lane,
                               (pr.is_reversed) ?
                               maliput::api::LaneEnd::kStart :
                               maliput::api::LaneEnd::kFinish);
}

bool IsConfluentWith(const maliput::api::LaneEnd& a,
                     const maliput::api::LaneEnd& b) {
  // TODO(maddog@tri.global)  This object equality isn't necessarily part of
  //                          the GetConfluentBranches() contract.  Should it
  //                          be?  (Everything is easy until you start tiling.)
  return (a.lane->GetConfluentBranches(a.end) ==
          b.lane->GetConfluentBranches(b.end));
}


LaneRelation DetermineLaneRelation(const PathRecord& pra,
                                   const PathRecord& prb) {
  // Assuming that the lanes described by pra and prb belong to the same
  // junction, determine how they are related.
  DRAKE_DEMAND(pra.lane->segment()->junction() ==
               prb.lane->segment()->junction());

  // (Below, "originate" and "terminate" are relative to the direction of
  // travel on a lane, e.g. a 'reversed' lane originates at LaneEnd::kFinish.)
  //
  // The options are:
  //
  // 0) If they do not share a BranchPoint, they merely intersect.
  //     #    *----\ /--->*    #
  //     #          \          #
  //     #    *----/ \--->*    #
  //
  // 1) If they terminate at the same BranchPoint, they merge.
  //     #    *---->----\      #
  //     #               *     #
  //     #    *---->----/      #
  //
  // 2) If they originate at the same BranchPoint, they split.
  //     #     /---->----*     #
  //     #    *                #
  //     #     \---->----*     #
  //
  // 3) They could originate at a shared BranchPoint, *and* terminate at
  //    another shared BranchPoint, i.e., "split-merge".
  //     #    /---->----\      #
  //     #   *           *     #
  //     #    \---->----/      #
  //
  // 4) They could originate and terminate at a single shared BranchPoint,
  //    i.e., "tangent loops".
  //     #   /-------------\   #
  //     #   \-->--\ /-->--/   #
  //     #          *          #
  //     #   /-->--/ \-->--\   #
  //     #   \-------------/   #
  //
  // Shared-BranchPoint configurations that have opposing travel-directions
  // are considered intersecting --- because if two vehicles occupy such a
  // configuration they will collide.
  bool is_split = IsConfluentWith(GetStartEnd(pra), GetStartEnd(prb));
  bool is_merge = IsConfluentWith(GetFinishEnd(pra), GetFinishEnd(prb));

  if (is_split && is_merge) {
    if (pra.lane->GetBranchPoint(maliput::api::LaneEnd::kStart) ==
        pra.lane->GetBranchPoint(maliput::api::LaneEnd::kFinish)) {
      return kTangentLoops;
    }
    return kSplitMerge;
  } else if (is_split) {
    return kSplit;
  } else if (is_merge) {
    return kMerge;
  } else {
    return kIntersection;
  }
}


void AssessJunctions(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs) {
  // "For each Junction, a list of all TimeBox's with a Lane in that Junction."
  std::map<const maliput::api::Junction*,
           std::vector<TimeBox>> boxes_by_junction;
  // "For each car (identified by its index), a list of all Junctions which
  //  its path will traverse (within some horizon)."
  std::map<int, std::vector<const maliput::api::Junction*>> junctions_by_car;

  IndexJunctions(source_states, paths, &boxes_by_junction, &junctions_by_car);
  MeasureJunctions(source_states, paths, boxes_by_junction, junctions_by_car,
                   oracle_outputs);
}


void IndexJunctions(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    std::map<const api::Junction*, std::vector<TimeBox>>* boxes_by_junction,
    std::map<int, std::vector<const api::Junction*>>* junctions_by_car) {
  // For each car, measure/record the 'time-in' and 'time-out' to all junctions
  // expected to be traversed by the explored path.
  for (size_t car_index = 0; car_index < source_states.size(); ++car_index) {
    // state of the i-th car
    const SourceState& self = source_states[car_index];
    // path of the i-th car
    const std::vector<PathRecord>& path = paths[car_index];
    // Sanity check:  current lane is the first lane in the path.
    DRAKE_DEMAND(self.rp.lane == path[0].lane);

    double s_current = 0.;
    bool is_first = true;
    for (const PathRecord& pr : path) {
      double pr_length{};
      if (!pr.is_reversed) {
        const double s0 = (is_first) ? self.rp.pos.s : 0.;
        pr_length = pr.lane->length() - s0;
      } else {
        const double s0 = (is_first) ? self.rp.pos.s : pr.lane->length();
        pr_length = s0;
      }
      DRAKE_DEMAND(pr_length > 0.);
      const double s_in = s_current;
      s_current += pr_length;
      const double s_out = s_current;
      // An exactly-stopped car would never reach the junction; it is more
      // stable to pretend that it is travelling very, very slowly (that way
      // the closest exactly-stopped car will look like it is going to
      // get there first).
      const double kNonZeroSpeed = 1e-12;
      const double speed = std::max(self.circuit_s_speed, kNonZeroSpeed);
      // TODO(maddog@tri.global)  Time in/out should probably account
      //                          for the length of the vehicle, too.
      const double time_in = s_in / speed;
      const double time_out = s_out / speed;
      const TimeBox time_box{car_index, pr, time_in, time_out, s_in, s_out};

      const maliput::api::Junction* junction =
          pr.lane->segment()->junction();
      (*boxes_by_junction)[junction].push_back(time_box);
      (*junctions_by_car)[car_index].push_back(junction);

      is_first = false;
    }
  }
}


void MeasureJunctions(
    const std::vector<SourceState>& source_states,
    const std::vector<std::vector<PathRecord>>& paths,
    const std::map<const api::Junction*,
    std::vector<TimeBox>>& boxes_by_junction,
    const std::map<int, std::vector<const api::Junction*>>& junctions_by_car,
    const std::vector<EndlessRoadOracleOutput<double>*>& oracle_outputs) {
  // For each car, find cars which are expected to occupy the same junction
  // at the same time.
  for (size_t car_index = 0; car_index < source_states.size(); ++car_index) {
    const SourceState& self = source_states[car_index];

    double might_collide_at = kEnormousDistance;
    double maybe_diff_vel{};

    // Iterate over the junctions which car 'car_index' participates in.
    for (const auto& junction : junctions_by_car.at(car_index)) {
      // Find the TimeBox for this car at this junction.
      const TimeBox self_box = [&]() {
        // TODO(maddog)  Linear search is kind of dumb.
        // TODO(maddog)  There are potentially multiple TimeBoxes for the same
        //               car at the same junction, if the car circles back
        //               through the junction within the horizon.
        for (const TimeBox& box : boxes_by_junction.at(junction)) {
          if (box.car_index == car_index) { return box; }
        }
        DRAKE_ABORT();
      }();
      DRAKE_DEMAND(self_box.time_in < self_box.time_out);

      const double kMergeHorizonCarLengths = 10.;
      const double kMergeHorizonSeconds = 2.;
      const double merge_horizon =
          (kMergeHorizonCarLengths * kCarLength) +
          (kMergeHorizonSeconds * self.circuit_s_speed);

      // If self is *already in* the intersection, then skip it.
      // (E.g., keep going and get out of the intersection!)
      if (self_box.time_in == 0.) { continue; }

      // Are there any intersecting TimeBoxes from other cars at this junction?
      for (const auto& other_box : boxes_by_junction.at(junction)) {
        // Skip self (car 'car_index').
        if (other_box.car_index == car_index) {
          continue;
        }
        // Skip same lane (which cannot be an intersecting path).
        // TODO(maddog)  Actually process same-lane, thus subsume the work of
        //               AssessForwardPath().
        if (other_box.pr.lane == self_box.pr.lane) {
          continue;
        }
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
              maybe_diff_vel = self.circuit_s_speed - 0.;
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
                  self.circuit_s_speed -
                  source_states[other_box.car_index].circuit_s_speed;
            }
            break;
          }
          case kSplit: {
            // TODO(maddog) Handle split.
            continue;
          }
          case kSplitMerge: {
            // TODO(maddog) Handle split-merge.
            DRAKE_ABORT();
          }
          case kTangentLoops: {
            // TODO(maddog) Handle tangent-loops.
            DRAKE_ABORT();
          }
        }

        // TODO(maddog) Compare speeds?  We really want the slower car
        //              to go slower, and faster faster, right?
        // TODO(maddog) Skip cars coming from left.  (necessary?)
      }
    }

    if (might_collide_at < oracle_outputs[car_index]->net_delta_sigma()) {
      oracle_outputs[car_index]->set_net_delta_sigma(might_collide_at);
      oracle_outputs[car_index]->set_delta_sigma_dot(maybe_diff_vel);
    }
  }
}

}  // namespace internal


// These instantiations must match the API documentation in
// endless_road_oracle.h.
template class EndlessRoadOracle<double>;
// TODO(maddog@tri.global)  Eventually, AutoDiffXd, etc.

}  // namespace automotive
}  // namespace drake
