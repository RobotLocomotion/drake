#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic_expression.h"
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
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class Demultiplexer : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Demultiplexer)

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

 private:
  // Sets the i-th output port to the value of the i-th component of the input
  // port.
  void CopyToOutput(const Context<T>& context, OutputPortIndex port_index,
                    BasicVector<T>* output) const;

  // Returns a Demultiplexer<symbolic::Expression> with the same dimensions as
  // this Demultiplexer.
  Demultiplexer<symbolic::Expression>* DoToSymbolic() const override;
};

}  // namespace systems
}  // namespace drake
