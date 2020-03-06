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
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class Demultiplexer final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Demultiplexer)

  /// Constructs %Demultiplexer with one vector valued output ports with sizes
  /// specified as the vector @p output_ports_sizes. The number of output ports
  /// is the length of this vector. The size of each output port is the value of
  /// the corresponding element of the vector @p output_ports_sizes.
  ///
  /// @throws std::logic_error if @p output_ports_sizes is a zero length vector.
  /// @throws std::logic_error if any element of the @p output_ports_sizes is
  /// zero. Therefore, the Demultiplexer does not allow zero size output ports.
  ///
  /// @param output_ports_sizes Contains the sizes of each output port. The
  /// number of output ports is determined by the length of @p
  /// output_ports_sizes. The accumulative value of the all the values in @p
  /// output_ports_sizes will be the size of the input port.
  explicit Demultiplexer(const std::vector<int>& output_ports_sizes);

  /// Constructs %Demultiplexer with one vector valued input port of size
  /// @p size and vector valued output ports of size @p output_ports_size.
  ///
  /// @throws std::logic_error if output_ports_sizes can not exactly divide @p
  /// size. The number of output ports is therefore `size / output_ports_size`.
  ///
  /// @param size is the size of the input signal to be demultiplexed into its
  /// individual components.
  /// @param output_ports_size The size of the output ports. @p size must be
  /// a multiple of @p output_ports_size.
  explicit Demultiplexer(int size, int output_ports_size = 1);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit Demultiplexer(const Demultiplexer<U>&);

  const std::vector<int>& get_output_ports_sizes() const {
    return output_ports_sizes_;
  }

 private:
  // Sets the port_index-th output port value.
  void CopyToOutput(const Context<T>& context, OutputPortIndex port_index,
                    BasicVector<T>* output) const;

  // Return a std vector with size of 'size / output_ports_size' and all values
  // as @p output_ports_sizes.
  static const std::vector<int> CalcOutputPortsSizes(int size,
                                                     int output_ports_size) {
    // The size must be a multiple of output_ports_size.
    DRAKE_DEMAND(size % output_ports_size == 0);
    const int num_output_ports = size / output_ports_size;
    return std::vector<int>(num_output_ports, output_ports_size);
  }

  // Return a std vector with the same size as the input @p output_ports_sizes.
  // The values are calculated as the start index of the output port inside the
  // input port vector.
  static const std::vector<int> CalcOutputPortsStart(
      const std::vector<int>& output_ports_sizes) {
    const int num_output_ports = output_ports_sizes.size();
    // Require the number of output ports should be greater than 1.
    DRAKE_DEMAND(num_output_ports >= 1);
    std::vector<int> output_ports_start;
    output_ports_start.resize(num_output_ports);

    output_ports_start[0] = 0;
    for (int i = 1; i < num_output_ports; i++) {
      output_ports_start[i] =
          output_ports_start[i - 1] + output_ports_sizes[i - 1];
    }
    return output_ports_start;
  }

  const std::vector<int> output_ports_sizes_;

  const std::vector<int> output_ports_start_;
};

}  // namespace systems
}  // namespace drake
