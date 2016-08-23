#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/primitives/pass_through.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// Wires up a diagram system for a PID controller.
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
template<typename T>
DiagramBuilder<T> MakePidController(const T& Kp, const T& Ki, int length);

/// An integrator for a continuous vector input.
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
template <typename T>
class PidController : public Diagram<T> {
 public:
  /// @param length is the size of the input port.
  explicit PidController(const T& Kp, const T& Ki, int length);
  ~PidController() override {}


  /// The input directly feeds through the proportional gain system.
  bool has_any_direct_feedthrough() const override { return true;}

  // Default implementation in Diagram<T>
  //std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override;

  // Default implementation in Diagram<T>
  //std::unique_ptr<SystemOutput<T>> AllocateOutput(
  //    const ContextBase<T>& context) const override;

  // Default implementation in Diagram<T>
  //void EvalOutput(const ContextBase<T>& context,
  //                SystemOutput<T>* output) const override;

  // This needs implementation in Diagram<T>!!!
  //std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override;

  // This needs implementation in Diagram<T>!!!
  //void EvalTimeDerivatives(const ContextBase<T>& context,
  //                         ContinuousState<T>* derivatives) const override;

 private:
  const int length_{0};
  std::unique_ptr<Adder<T>> adder_;
  std::unique_ptr<Integrator<T>> integrator_;
  std::unique_ptr<PassThrough<T>> pass_through_;
  std::unique_ptr<Gain<T>> proportional_gain_;
  std::unique_ptr<Gain<T>> integrator_gain_;
};

}  // namespace systems
}  // namespace drake
