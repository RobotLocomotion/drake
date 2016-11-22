#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {

/// Simple model to capture the quantization and calibration offset effects
/// of a rotary encoder.  Consider combining this with a zero-order hold system
/// to capture the sampled-data effects.
///
/// @ingroup sensor_systems
template <typename T>
class RotaryEncoders : public systems::LeafSystem<T> {
 public:
  /// Specifies one ticks_per_revolution count for every element of the input
  /// port.
  explicit RotaryEncoders(
      const std::vector<unsigned int>& ticks_per_revolution);

  /// Provides arguments to select particular indices from the input signal to
  /// use in the output.
  /// @param input_port_size Dimension of the expected input signal
  /// @param input_vector_indices List of indices
  RotaryEncoders(const int input_port_size,
                 const std::vector<unsigned int>& input_vector_indices);

  /// Sets both the quantization parameters and the selector parameters.
  RotaryEncoders(const int input_port_size,
                 const std::vector<unsigned int>& input_vector_indices,
                 const std::vector<unsigned int>& ticks_per_revolution);

  /// Outputs the transformed signal.
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  /// Calibration offsets are defined as parameters.
  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  /// Set the calibration offset parameters.
  void set_calibration_offsets(
      systems::Context<T>* context,
      const Eigen::Ref<VectorX<T>>& calibration_offsets) const;

  /// Retreive the calibration offset parameters.
  Eigen::VectorBlock<const VectorX<T>> get_calibration_offsets(
      const systems::Context<T>& context) const;

 private:
  const unsigned int num_encoders_;          /// Dimension of the output port.
  const std::vector<unsigned int> indices_;  /// Selects from the input port.
  const std::vector<unsigned int> ticks_per_revolution_;  /// For quantization.
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
