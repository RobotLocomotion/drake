#include "drake/systems/sensors/test/accelerometer_test/accelerometer_xdot_filter.h"

#include <memory>

#include <Eigen/Dense>

using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace sensors {

AccelerometerXdotFilter::AccelerometerXdotFilter(int port_size)
    : port_size_(port_size) {
  input_port_index_ =
      DeclareInputPort(kVectorValued, port_size).get_index();
  output_port_index_ =
      DeclareOutputPort(kVectorValued, port_size).get_index();
}

std::unique_ptr<BasicVector<double>>
AccelerometerXdotFilter::AllocateOutputVector(
    const OutputPortDescriptor<double>& descriptor) const {
  return std::make_unique<BasicVector<double>>(port_size_);
}

void AccelerometerXdotFilter::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));

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
