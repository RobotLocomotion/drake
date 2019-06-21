#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This system splits a vector valued signal on its input into multiple
/// outputs.
///
/// The input to this system directly feeds through to its output.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class Demultiplexer final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Demultiplexer)

  /// Constructs %Demultiplexer with one vector valued input port of size
  /// @p size and vector valued output ports with sizes specified as the vector
  /// @p output_ports_sizes. The number of output ports is the length of this
  /// vector.
  ///
  /// @p size must be greater than zero.
  ///
  /// @param size is the size of the input signal to be demultiplexed into its
  /// individual components.
  /// @param output_ports_sizes Contains the sizes of each output port. The
  /// number of output ports equal to the length of @p output_ports_sizes. The
  /// accumulative value of the all the values in
  /// @p output_ports_sizes must be equal to @p size.
  explicit Demultiplexer(int size, const std::vector<int>& output_ports_sizes);

  /// Constructs %Demultiplexer with one vector valued input port of size
  /// @p size and vector valued output ports of size @p output_ports_size.
  ///
  /// @p output_ports_sizes must exactly divide @p size. Otherwise this
  /// constructor throws an exception. The number of output ports is therefore
  /// `size / output_ports_size`.
  ///
  /// @param size is the size of the input signal to be demultiplexed into its
  /// individual components.
  /// @param output_ports_size The size of the output ports. @p size must be
  /// a multiple of @p output_ports_size.
  explicit Demultiplexer(int size, int output_ports_size = 1);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit Demultiplexer(const Demultiplexer<U>&);

  const std::vector<int> get_output_ports_sizes() const {
    return output_ports_sizes_;
  }

 private:
  // Sets the port_index-th output port value.
  void CopyToOutput(const Context<T>& context, OutputPortIndex port_index,
                    BasicVector<T>* output) const;

  // Return a std vector with size of 'size / output_ports_size' and all values
  // as @p output_ports_size.
  static const std::vector<int> CalOutputPortsSizes(int size,
                                                    int output_ports_size) {
    // The size must be a multiple of output_ports_sizes.
    DRAKE_DEMAND(size % output_ports_size == 0);
    const int num_output_ports = size / output_ports_size;
    return std::vector<int>(num_output_ports, output_ports_size);
  }

  const std::vector<int> output_ports_sizes_;
};

}  // namespace systems
}  // namespace drake
