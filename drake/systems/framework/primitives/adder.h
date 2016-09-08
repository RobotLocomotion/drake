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

/// An adder for arbitrarily many inputs of equal length.
/// @tparam T The type of mathematical object being added.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
template <typename T>
class Adder : public LeafSystem<T> {
 public:
  /// @param num_inputs is the number of input ports to be added.
  /// @param length is the size of each input port.
  Adder(int num_inputs, int length);

  /// All inputs to this system are directly fed through to its output.
  bool has_any_direct_feedthrough() const override { return true; }

  /// Sums the input ports into the output port. If the input ports are not
  /// of number num_inputs_ or size length_, std::runtime_error will be thrown.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;
};

}  // namespace systems
}  // namespace drake
