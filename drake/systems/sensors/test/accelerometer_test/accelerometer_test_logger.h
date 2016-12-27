#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A sink block that logs the most recent plant state and the accelerometer
/// reading.
///
/// @ingroup primitive_systems
class AccelerometerTestLogger : public LeafSystem<double> {
 public:
  /// Construct the accelerometer test logger system.
  //
  /// @param plant_state_size The size of the plant's state.
  explicit AccelerometerTestLogger(int plant_state_size);

  // Non-copyable.
  AccelerometerTestLogger(const AccelerometerTestLogger&) = delete;
  AccelerometerTestLogger& operator=(const AccelerometerTestLogger&)
      = delete;

  void enable_log_to_console() { log_to_console_ = true; }

  Eigen::VectorXd get_plant_state() const { return plant_state_; }
  Eigen::VectorXd get_plant_state_derivative() const {
    return plant_state_derivative_;
  }
  Eigen::VectorXd get_acceleration() const { return acceleration_; }

  const InputPortDescriptor<double>& get_plant_state_input_port() const;
  const InputPortDescriptor<double>& get_plant_state_derivative_input_port()
      const;
  const InputPortDescriptor<double>& get_acceleration_input_port() const;

 private:
  // No output.
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  // Logging is done in this method.
  void DoPublish(const Context<double>& context) const override;

  // const int batch_allocation_size_{1000};

  // Use mutable variables to hold the logged data.
  // mutable int num_samples_{0};
  // mutable VectorX<T> sample_times_;
  // mutable MatrixX<T> data_;

  bool log_to_console_{false};
  int plant_state_derivative_port_index_{};
  int plant_state_port_index_{};
  int acceleration_port_index_{};

  mutable Eigen::VectorXd plant_state_;
  mutable Eigen::VectorXd plant_state_derivative_;
  mutable Eigen::Vector3d acceleration_;
};

}  // namespace systems
}  // namespace drake
