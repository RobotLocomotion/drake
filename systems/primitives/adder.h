#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace systems {

/// An adder for arbitrarily many inputs of equal size.
///
/// @system
/// name: Adder
/// input_ports:
/// - u0
/// - ...
/// - u(N-1)
/// output_ports:
/// - sum
/// @endsystem
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class Adder final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Adder)

  /// Construct an %Adder System.
  /// @param num_inputs is the number of input ports to be added.
  /// @param size number of elements in each input and output signal.
  Adder(int num_inputs, int size);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Adder(const Adder<U>&);

 private:
  // Sums the input ports into a value suitable for the output port. If the
  // input ports are not the appropriate count or size, std::runtime_error will
  // be thrown.
  void CalcSum(const Context<T>& context, BasicVector<T>* sum) const;
};

}  // namespace systems
}  // namespace drake
