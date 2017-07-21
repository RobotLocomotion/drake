#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/siso_vector_system.h"

namespace drake {
namespace systems {
namespace sensors {

/// Simple model to capture the quantization and calibration offset effects
/// of a rotary encoder.  Consider combining this with a ZeroOrderHold system
/// to capture the sampled-data effects.
///
/// The inputs to this system are assumed to be in radians, and the outputs of
/// the system are also in radians.
///
/// @ingroup sensor_systems
template <typename T>
class RotaryEncoders : public SisoVectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RotaryEncoders)

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

  /// Transmogrification constructor.
  template <typename U>
  RotaryEncoders(const TransmogrifierTag&, const RotaryEncoders<U>&);

  /// Calibration offsets are defined as parameters.
  std::unique_ptr<Parameters<T>> AllocateParameters() const override;

  /// Set the calibration offset parameters.
  void set_calibration_offsets(
      Context<T>* context,
      const Eigen::Ref<VectorX<T>>& calibration_offsets) const;

  /// Retreive the calibration offset parameters.
  Eigen::VectorBlock<const VectorX<T>> get_calibration_offsets(
      const Context<T>& context) const;

  /// Returns the selection list.  For return_value[i] == j, the output port's
  /// i'th element comes from the input port's j'th element.  This value will
  /// be non-empty even if a quantization-only constructor was used (in which
  /// case it would be an identity vector with return_value[i] == i).
  const std::vector<int>& input_vector_indices() const;

  /// Returns the quantization configuration for all of the input port's
  /// indices, or else an empty vector if quantization is disabled.
  const std::vector<int>& ticks_per_revolution() const;

 private:
  // Outputs the transformed signal.
  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

  void SetDefaultParameters(const LeafContext<T>& context,
                            Parameters<T>* params) const override;

  const int num_encoders_{0};       // Dimension of the output port.
  const std::vector<int> indices_;  // Selects from the input port.
  const std::vector<int> ticks_per_revolution_;  // For quantization.
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
