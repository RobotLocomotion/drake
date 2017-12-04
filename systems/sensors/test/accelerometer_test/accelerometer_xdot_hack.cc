#include "drake/systems/sensors/test/accelerometer_test/accelerometer_xdot_hack.h"

#include <memory>

#include <Eigen/Dense>

using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace sensors {

AccelerometerXdotHack::AccelerometerXdotHack(int port_size) {
  input_port_index_ = DeclareInputPort(kVectorValued, port_size).get_index();
  output_port_index_ =
      DeclareVectorOutputPort(BasicVector<double>(port_size),
                              &AccelerometerXdotHack::CalcXdotOutput)
          .get_index();
}

void AccelerometerXdotHack::CalcXdotOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output_vector) const {
  VectorXd xdot = this->EvalEigenVectorInput(context, input_port_index_);
  if (!xdot.allFinite()) {
    xdot = VectorXd::Zero(xdot.size());
  }
  output_vector->SetFromVector(xdot);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
