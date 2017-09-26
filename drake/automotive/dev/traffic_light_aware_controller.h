#pragma once

#include <math.h>
#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// This controller takes as input an acceleration from another controller,
/// and also looks at the traffic light. If the traffic light is "open",
/// this controller simply forwards the acceleration command from the other
/// controller; otherwise it overrides the acceleration with a braking
/// command.
///
/// Currently, supported value of T is double.
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

}  // namespace drake
}  // namespace automotive
