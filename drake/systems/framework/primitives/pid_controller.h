#pragma once

#include <memory>

#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/primitives/pass_through.h"

namespace drake {
namespace systems {

template <typename T>
class PidController : public Diagram<T> {
 public:
  /// @param length is the size of the input port.
  PidController(const T& Kp, const T& Ki, int length);

  ~PidController() override {}

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;

  /// Sets @p context to a default state in which the integral of the
  /// controller is zero.
  void SetDefaultState(ContextBase<T>* context) const;

  /// Sets the integral of the %PidController to zero.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(ContextBase<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

 private:
  std::unique_ptr<Adder<T>> adder_;
  std::unique_ptr<Integrator<T>> integrator_;
  std::unique_ptr<PassThrough<T>> pass_through_;
  std::unique_ptr<Gain<T>> proportional_gain_;
  std::unique_ptr<Gain<T>> integrator_gain_;
};

}  // namespace systems
}  // namespace drake
