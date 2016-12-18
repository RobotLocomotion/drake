#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This system splits a vector valued signal in its inputs of size `size`
/// into `size` output scalar valued signals.
/// The input to this system directly feeds through to its output.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class Demultiplexer : public LeafSystem<T> {
 public:
  /// Constructs %Demultiplexer with one vector valued input port of size
  /// @p size and vector valued output ports of size @p output_ports_sizes.
  ///
  /// @p output_ports_sizes must exactly divide @p size. Otherwise this
  /// constructor throws an exception. The number of output ports is therefore
  /// `size / output_ports_sizes`.
  ///
  /// @param size is the size of the input signal to be demultiplexed into its
  /// individual components.
  /// @param output_ports_sizes The size of the output ports. @p length must be
  /// a multiple of @p output_ports_sizes.
  explicit Demultiplexer(int size, int output_ports_sizes = 1);

  /// Sets the i-th output port to the value of the i-th component of the input
  /// port.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;
};

}  // namespace systems
}  // namespace drake
