#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A gain block with input `u` and output `y = k*u` with `k` a constant.
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
  /// Constructs a %Gain system.
  /// @param k the gain constant so that `y = k*u`.
  /// @param size number of elements in the signal to be processed.
  Gain(const T& k, int size);

  /// Returns the gain constant.
  const T& get_gain() const;

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
  const T gain_;
};

}  // namespace systems
}  // namespace drake
