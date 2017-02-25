#include "drake/automotive/powered_bicycle.h"

#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

template <typename T>
PoweredBicycle<T>::PoweredBicycle() : systems::Diagram<T>() {
  // Time constant approximated from Section 4.5 (p. 45) of
  // Lind, Simon. Powertrain modeling for realtime simulation, 2014.
  //    http://liu.diva-portal.org/smash/get/diva2:763494/
  const double kPowertrainTimeConstant = 0.2; /* [s] */
  const double kPowertrainGain = 1.;          /* [N] */
  systems::DiagramBuilder<T> builder;

  // Instantiate the bicycle and the powertrain system models.
  bike_ = builder.AddSystem(std::make_unique<Bicycle<T>>());
  power_ = builder.AddSystem(std::make_unique<SimplePowertrain<T>>(
      kPowertrainTimeConstant, kPowertrainGain));

  builder.Connect(power_->get_force_output_port(),
                  bike_->get_force_input_port());

  builder.ExportInput(bike_->get_steering_input_port());
  builder.ExportInput(power_->get_throttle_input_port());

  builder.ExportOutput(power_->get_output_port());
  builder.ExportOutput(bike_->get_state_output_port());

  builder.BuildInto(this);
}

// These instantiations must match the API documentation in powered_bicycle.h.
template class PoweredBicycle<double>;
template class PoweredBicycle<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
