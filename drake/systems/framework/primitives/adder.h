#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// An adder for arbitrarily many inputs of equal size.
/// @ingroup primitive_systems
/// @tparam T The type of mathematical object being added.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
template <typename T>
class Adder : public LeafSystem<T> {
 public:
  /// @param num_inputs is the number of input ports to be added.
  /// @param size number of elements in each input and output signal.
  Adder(int num_inputs, int size);

  /// All inputs to this system are directly fed through to its output.
  bool has_any_direct_feedthrough() const override { return true; }

  /// Returns the output port.
  const SystemPortDescriptor<T>& get_output_port() const;

  /// Sums the input ports into the output port. If the input ports are not
  /// the appropriate count or size, std::runtime_error will be thrown.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;
};

}  // namespace systems
}  // namespace drake
