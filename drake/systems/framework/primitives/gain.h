#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A gain block with input `u` and output `y = k * u` with `k` a constant.
/// The input to this system directly feeds through to its output.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in drakeSystemFramework.
///
/// To use other specific scalar types see gain-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup primitive_systems
template <typename T>
class Gain : public LeafSystem<T> {
 public:
  /// Constructs a %Gain system where the same gain is applied to every input
  /// value.
  ///
  /// @param[in] k the gain constant so that `y = k * u`.
  /// @param[in] size number of elements in the signal to be processed.
  Gain(const T& k, int size);

  /// Constructs a %Gain system where different gains can be applied to each
  /// input value.
  ///
  /// @param[in] k the gain vector constants so that `y_i = k_i * u_i` where
  /// subscript `i` indicates the i-th element of the vector.
  explicit Gain(const VectorX<T>& k);

  /// Returns the gain constant. This method should only be called if the gain
  /// can be represented as a scalar value, i.e., every element in the gain
  /// vector is the same. It will abort if the gain cannot be represented as a
  /// single scalar value.
  const T& get_gain() const;

  /// Returns the gain vector constant.
  const VectorX<T>& get_gain_vector() const;

  /// Sets the output port value to the product of the gain and the input port
  /// value. The gain is specified in the constructor.
  /// If number of connected input or output ports differs from one or, the
  /// input ports are not the correct size, std::runtime_error will be thrown.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

  /// Returns the input port.
  const SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const SystemPortDescriptor<T>& get_output_port() const;

 private:
  // TODO(amcastro-tri): move gain_ to System<T>::Parameter.
  const VectorX<T> k_;
};

}  // namespace systems
}  // namespace drake
