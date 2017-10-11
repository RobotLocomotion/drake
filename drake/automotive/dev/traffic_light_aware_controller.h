#pragma once

#include <cmath>
#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// This controller takes as input an acceleration command and also looks at
/// the traffic light and car position. If the traffic light is "open", or
/// if the car is outside the radius of the traffic light, this controller
/// simply forwards the acceleration command by placing it in its output
/// port. Otherwise, if the signal is "closed" and the car is at the edge
/// of its radius, this controller overrides the acceleration with a
/// braking command.
///
/// Currently, the only supported value of T is double.
// TODO(nikos-tri) Support AutoDiffXd and Symbolic.

template <typename T>
class TrafficLightAwareController : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLightAwareController)

  TrafficLightAwareController();

  const systems::InputPortDescriptor<T>& car_state() const;
  const systems::InputPortDescriptor<T>& other_controller_acceleration() const;
  const systems::InputPortDescriptor<T>& traffic_light_input() const;
  const systems::OutputPort<T>& acceleration_output() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;

  // Indices for input / output ports.
  const int car_state_input_index_{};
  const int acceleration_input_index_{};
  const int traffic_light_input_index_{};
  const int output_index_{};

  VectorX<T> ReadParameters(const systems::Context<T>& context) const;
  const VectorX<T> ReadInput(const systems::Context<T>& context,
                             int input_index) const;
};

}  // namespace automotive
}  // namespace drake
