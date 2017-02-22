#include "drake/automotive/powered_bicycle.h"

#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

template <typename T>
PoweredBicycle<T>::PoweredBicycle() : systems::Diagram<T>() {
  const double kPowerTrainTimeConstant = 0.4;
  const double kPowerTrainGain = 1.;
  systems::DiagramBuilder<T> builder;

  // Instantiate the bicycle and the powertrain system models.
  bike_ = builder.AddSystem(std::make_unique<Bicycle<T>>());
  power_ = builder.AddSystem(std::make_unique<SimplePowerTrain<T>>(
      kPowerTrainTimeConstant, kPowerTrainGain));

  builder.Connect(power_->get_force_output_port(),
                  bike_->get_force_input_port());

  builder.ExportInput(bike_->get_steering_input_port());  // Exports to port 0.
  builder.ExportInput(power_->get_throttle_input_port());  // Exports to port 1.
  builder.ExportOutput(bike_->get_state_output_port());  // Exports to port 0.

  builder.BuildInto(this);
}

// These instantiations must match the API documentation in powered_bicycle.h.
template class PoweredBicycle<double>;
template class PoweredBicycle<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
