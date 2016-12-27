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

AccelerometerTestLogger::AccelerometerTestLogger(int plant_state_size)
    : plant_state_(plant_state_size) {
  DRAKE_DEMAND(plant_state_size > 0);
  plant_state_port_index_ =
      this->DeclareInputPort(kVectorValued, plant_state_size).get_index();
  plant_state_derivative_port_index_ =
      this->DeclareInputPort(kVectorValued, plant_state_size).get_index();
  acceleration_port_index_ =
      this->DeclareInputPort(kVectorValued, Accelerometer::kNumMeasurements)
          .get_index();
}

void AccelerometerTestLogger::DoPublish(const Context<double>& context)
    const {
  double current_time = context.get_time();

  // Record time and input to the num_samples position.
  plant_state_ =
      this->EvalVectorInput(context, plant_state_port_index_)->get_value();
  plant_state_derivative_ =
      this->EvalVectorInput(context, plant_state_derivative_port_index_)->
          get_value();
  acceleration_ =
      this->EvalVectorInput(context, acceleration_port_index_)->get_value();

  if (log_to_console_) {
    std::stringstream buffer;
    buffer << "AccelerometerTestLogger::DoPublish:\n"
              "  - time: " << std::to_string(current_time) << "\n"
              "  - plant_state = " << plant_state_.transpose() << "\n"
              "  - plant_state_derivative = "
                  << plant_state_derivative_.transpose() << "\n"
              "  - acceleration = " << acceleration_.transpose();
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

}  // namespace systems
}  // namespace drake
