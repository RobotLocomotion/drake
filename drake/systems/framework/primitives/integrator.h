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

/// An integrator for a continuous vector input.
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
template <typename T>
class Integrator : public System<T> {
 public:
  /// @param length is the size of the input port.
  explicit Integrator(int length);
  ~Integrator() override {}

  /// Integrator's output is not a direct feedthrough of its input.
  bool has_any_direct_feedthrough() const override { return false;}

  /// Allocates the number of input ports specified in the constructor.
  /// Allocates no state.
  std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override;

  /// Allocates one output port of the width specified in the constructor.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const override;

  /// Integrates the input ports into the output port. If the input ports are
  /// not of the length specified in the constructor, std::runtime_error will
  /// be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override;

  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override;

  void set_name(const std::string& name) { name_ = name; }
  std::string get_name() const override { return name_; }

 private:
  std::string name_;
  const int length_{0};
};

}  // namespace systems
}  // namespace drake
