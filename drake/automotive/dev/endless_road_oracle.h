#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

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
/// with state expressed in LANE-space on an InfiniteCircuitRoad (a maliput
/// road network that only has a single Lane and infinite longitudinal extent).
///
/// (Elevation-above-road 'h' is implicitly zero, too.)
///
/// For each car, the basic sensing performed is to determine both
///  * the net (bumper-to-bumper) longitudinal distance and
///  * the differential longitudinal speed
/// to the closest car ahead in the lane.  There are a couple of special
/// enhancements as well:
///  * Lanes that are merging into a car's current lane (within a horizon)
///    are inspected as well, to allow a car to coordinate with any merging
///    cars.
///  * Intersections which are anticipated to be occupied by a crossing
///    vehicle will be treated as blocked until clear.
///
/// The underlying algorithm is independent for each vehicle, in the sense
/// that for each Sensing-Car:
///
/// <pre>
///   Sensor-Output = F(State-of-Sensing-Car, State-of-All-Other-Cars)
/// </pre>
///
/// Even though there are no dependencies between sensor outputs (e.g.,
/// explicitly mediated turntaking), this class computes the sensor-output for
/// all cars at once for the sake of efficiency.
///
/// state vector:  none; stateless
///
/// input vector:
/// * an EndlessRoadCarState port for each EndlessRoadCar serviced
///
/// output vector:
/// * an EndlessRoadOracleOutput port for each EndlessRoadCar serviced
///
/// @tparam T must support certain arithmetic operations.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// @ingroup automotive_systems
template <typename T>
class EndlessRoadOracle : public systems::LeafSystem<T> {
 public:
  /// Constructor.
  ///
  /// @param road  the InfiniteCircuitRoad on which the cars are driving,
  ///              i.e., on which the EndlessRoadCarState is defined
  /// @param num_cars  the number of EndlessRoadCars to service, which
  ///                  determines the number of input and output ports in
  ///                  this EndlessRoadOracle
  EndlessRoadOracle(const maliput::utility::InfiniteCircuitRoad* road,
                    const int num_cars);

 public:
  // System<T> overrides
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;

 private:
  void ImplCalcOutput(
      const std::vector<const EndlessRoadCarState<T>*>& car_inputs,
      const std::vector<EndlessRoadOracleOutput<T>*>& oracle_outputs) const;

  const maliput::utility::InfiniteCircuitRoad* const road_;
  const int num_cars_;
};

}  // namespace automotive
}  // namespace drake
