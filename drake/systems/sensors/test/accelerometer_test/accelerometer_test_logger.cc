#include "drake/systems/sensors/test/accelerometer_test/accelerometer_test_logger.h"

#include <sstream>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/sensors/accelerometer.h"

namespace drake {
namespace systems {

using sensors::Accelerometer;

AccelerometerTestLogger::AccelerometerTestLogger(int plant_state_size) {
  DRAKE_DEMAND(plant_state_size > 0);
  plant_state_port_index_ =
      this->DeclareInputPort(kVectorValued, plant_state_size).get_index();
  plant_state_derivative_port_index_ =
      this->DeclareInputPort(kVectorValued, plant_state_size).get_index();
  acceleration_port_index_ =
      this->DeclareInputPort(kVectorValued, 3).get_index();
}

void AccelerometerTestLogger::DoPublish(const Context<double>& context)
    const {
  if (log_to_console_) {
    const auto plant_state =
        this->EvalVectorInput(context, plant_state_port_index_)->get_value();
    const auto plant_state_derivative =
        this->EvalVectorInput(context,
            plant_state_derivative_port_index_)->get_value();
    const auto acceleration =
        this->EvalVectorInput(context, acceleration_port_index_)->get_value();
    std::stringstream buffer;
    buffer << "AccelerometerTestLogger::DoPublish:\n"
              "  - time: " << std::to_string(context.get_time()) << "\n"
              "  - plant_state = " << plant_state.transpose() << "\n"
              "  - plant_state_derivative = "
                  << plant_state_derivative.transpose() << "\n"
              "  - acceleration = " << acceleration.transpose();
    drake::log()->info(buffer.str());
  }
}

const InputPortDescriptor<double>&
AccelerometerTestLogger::get_plant_state_input_port() const {
  return System<double>::get_input_port(plant_state_port_index_);
}

const InputPortDescriptor<double>&
AccelerometerTestLogger::get_plant_state_derivative_input_port() const {
  return System<double>::get_input_port(plant_state_derivative_port_index_);
}

const InputPortDescriptor<double>&
AccelerometerTestLogger::get_acceleration_input_port() const {
  return System<double>::get_input_port(acceleration_port_index_);
}

Eigen::VectorXd AccelerometerTestLogger::get_plant_state(
    const Context<double>& context) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  return this->EvalVectorInput(context, plant_state_port_index_)->get_value();
}

Eigen::VectorXd AccelerometerTestLogger::get_plant_state_derivative(
    const Context<double>& context) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  return this->EvalVectorInput(context,
          plant_state_derivative_port_index_)->get_value();
}

Eigen::VectorXd AccelerometerTestLogger::get_acceleration(
    const Context<double>& context) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  return this->EvalVectorInput(context, acceleration_port_index_)->get_value();
}

}  // namespace systems
}  // namespace drake
