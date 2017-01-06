#pragma once

#include <stdexcept>

#include <boost/optional.hpp>

#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/endless_road_oracle_output.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

namespace maliput {
namespace utility {
class InfiniteCircuitRoad;
}
}

namespace automotive {



/// An oracular simulated sensor for perceiving features of EndlessRoadCars
/// with state expressed in LANE-space on an InfiniteCircuitRoad,
/// i.e., a maliput road network that only has a single Lane and infinite
/// longitudinal extent.
///
/// (Elevation-above-road 'h' is implicitly zero, too.)
///
/// state vector
/// * planar LANE-space position:  s, r
/// * planar isometric LANE-space velocity:  (sigma, rho)-dot
///
/// input vector:
/// * Currently:  none (accelerations are just zero)
/// * Later:  planar isometric LANE-space acceleration: (sigma, rho)-ddot
///
/// output vector: same as state vector
template <typename T>
class EndlessRoadOracle : public systems::LeafSystem<T> {
 public:
  EndlessRoadOracle(const maliput::utility::InfiniteCircuitRoad* road,
                    const int num_cars);

 public:
  // System<T> overrides
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

 private:
  void DoEvalOutput(
      const std::vector<const EndlessRoadCarState<T>*>& car_inputs,
      std::vector<EndlessRoadOracleOutput<T>*>& oracle_outputs) const;

  const maliput::utility::InfiniteCircuitRoad* road_;
  const int num_cars_;
  // TODO(maddog)  Do we need to keep track of these here?
  std::vector<systems::SystemPortDescriptor<T>> inports_;
  std::vector<systems::SystemPortDescriptor<T>> outports_;

};

}  // namespace automotive
}  // namespace drake
