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
/// This block is direct feedthrough.
/// @tparam T The type of mathematical object being added.
template <typename T>
class Gain : public System<T> {
 public:
  /// @param k the gain constant so that `y = k*u`.
  /// @param length is the size of the signal to be processed.
  Gain(double k, int length);

  /// Allocates the number of input ports specified in the constructor.
  /// Allocates no state.
  std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override;

  /// Allocates one output port of the width specified in the constructor.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const override;

  /// Sums the input ports into the output port. If the input ports are not
  /// of number num_inputs_ or size length_, std::runtime_error will be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  // TODO(amcastro-tri): move gain_ to System<T>::Parameter.
  // gain_ should be double or of type T if in the context.
  const double gain_;
  const int length_;
};

}  // namespace systems
}  // namespace drake
