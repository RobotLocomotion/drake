#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// A gain block with input `u` and output `y = k*u` with k a constant.
/// The input to this system directly feeds through to its output.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class Gain : public System<T> {
 public:
  /// Constructs a Gain system with gain value @p k and input/output ports
  /// limited to have size @p length.
  /// @param k the gain constant so that `y = k*u`.
  /// @param length is the size of the signal to be processed.
  Gain(T k, int length);

  // Allocates default context for a Gain system.
  // Allocates no state.
  std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override;

  // Allocates one output port of the length specified in the constructor.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const override;

  /// Sets the output port value to the product of the gain and the input port
  /// value. The gain is specified in the constructor.
  /// If number of connected input or output ports differs from one or, the
  /// input ports are not of size length_, std::runtime_error will be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  // TODO(amcastro-tri): move gain_ to System<T>::Parameter.
  const T gain_;
  const int length_;
};

}  // namespace systems
}  // namespace drake
