#pragma once

#include <memory>

#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// A buffer system with input `u` and output `y = u`. This is mathematically
/// equivalent to a Gain system with gain equal to one. However this system
/// inccurs in no additional computational cost.
/// Usage case: this system can be used when needing to export an input in a
/// Diagram that needs to be wired to several inputs of its internal
/// sub-systems.
/// The input to this system directly feeds through to its output.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class Buffer : public System<T> {
 public:
  /// Constructs a Buffer system (`y = u`) with input/output ports limited to
  /// have size @p length.
  /// @param length is the size of the signal to be processed.
  explicit Buffer(int length);

  // Allocates default context for a Buffer system.
  // Allocates no state.
  std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override;

  // Allocates one output port of the length specified in the constructor.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const override;

  /// Sets the output port to equal the input port.
  /// If number of connected input or output ports differs from one or, the
  /// input ports are not of size length_, std::runtime_error will be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  // TODO(amcastro-tri): remove this internal parameter once #3102 is merged.
  const int length_;
};

}  // namespace systems
}  // namespace drake
