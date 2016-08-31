#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This system splits a vector valued signal in its inputs of size `length`
/// into `length` output scalar valued signals.
/// The input to this system directly feeds through to its output.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class Demultiplexer : public LeafSystem<T> {
 public:
  /// Constructs %Demultiplexer with one vector valued input port of size
  /// @p length and @p length scalar valued output ports.
  /// @param length is the size of the input signal to be demultiplexed into its
  /// individual components.
  explicit Demultiplexer(int length);

  /// Sets the i-th output port to the value of the i-th component of the input
  /// port.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;
};

}  // namespace systems
}  // namespace drake
