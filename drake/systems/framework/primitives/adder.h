#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// An adder for arbitrarily many inputs of equal length.
/// @tparam T The type of mathematical object being added.
template <typename T>
class Adder : public SystemInterface<T> {
 public:
  /// @param num_inputs is the number of input ports to be added.
  /// @param length is the size of each input port.
  Adder(int num_inputs, int length);

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

  void set_name(const std::string& name) { name_ = name; }
  std::string get_name() const override { return name_; }

 private:
  std::string name_;
  const int num_inputs_;
  const int length_;
};

}  // namespace systems
}  // namespace drake
