#include "drake/systems/sensors/test/accelerometer_test/accelerometer_xdot_hack.h"

#include <memory>

#include <Eigen/Dense>

using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace sensors {

AccelerometerXdotHack::AccelerometerXdotHack(int port_size) {
  input_port_index_ =
      DeclareInputPort(kVectorValued, port_size).get_index();
  output_port_index_ =
      DeclareOutputPort(kVectorValued, port_size).get_index();
}

void AccelerometerXdotHack::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  VectorXd xdot = this->EvalEigenVectorInput(context, input_port_index_);
  if (!xdot.allFinite()) {
    xdot = VectorXd::Zero(xdot.size());
  }
  BasicVector<double>* output_vector =
      output->GetMutableVectorData(output_port_index_);
  output_vector->SetFromVector(xdot);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
