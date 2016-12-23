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
/// The inputs to this system are assumed to be in radians, and the outputs of
/// the system are also in radians.
///
/// @ingroup sensor_systems
template <typename T>
class RotaryEncoders : public systems::LeafSystem<T> {
 public:
  /// Quantization-only constructor.  Specifies one ticks_per_revolution count
  /// for every element of the input port.
  explicit RotaryEncoders(const std::vector<int>& ticks_per_revolution);

  /// Selector-only constructor.  Provides arguments to select particular
  /// indices from the input signal to use in the output.  Since
  /// ticks_per_revolution is not being set, the outputs will NOT be quantized.
  /// @param input_port_size Dimension of the expected input signal
  /// @param input_vector_indices List of indices
  RotaryEncoders(int input_port_size,
                 const std::vector<int>& input_vector_indices);

  /// Quantization and Selector constructor.
  RotaryEncoders(int input_port_size,
                 const std::vector<int>& input_vector_indices,
                 const std::vector<int>& ticks_per_revolution);

  // Non-copyable.
  RotaryEncoders(const RotaryEncoders<T>&) = delete;
  RotaryEncoders& operator=(const RotaryEncoders<T>&) = delete;


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
  // Outputs the transformed signal.
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

  // System<T> override.
  RotaryEncoders<AutoDiffXd>* DoToAutoDiffXd() const override;

  const int num_encoders_{0};       // Dimension of the output port.
  const std::vector<int> indices_;  // Selects from the input port.
  const std::vector<int> ticks_per_revolution_;  // For quantization.
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
