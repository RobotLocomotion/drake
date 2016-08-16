#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A superclass template that extends System with some convenience utilities
/// that are not applicable to Diagrams.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class LeafSystem : public System<T> {
 public:
  ~LeafSystem() override {}

  /// Returns a default context, initialized with the correct
  /// numbers of concrete input ports and state variables for this System.
  std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override {
    std::unique_ptr<Context<T>> context(new Context<T>);
    ReserveInputs(context.get());
    ReserveState(context.get());
    return std::unique_ptr<ContextBase<T>>(context.release());
  }

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const override {
    std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
    for (const auto& descriptor : this->get_output_ports()) {
      std::unique_ptr<BasicVector<T>> data(
          new BasicVector<T>(descriptor.get_size()));
      std::unique_ptr<OutputPort<T>> port(new OutputPort<T>(std::move(data)));
      output->get_mutable_ports()->push_back(std::move(port));
    }
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

 protected:
  LeafSystem() {}

 private:
  /// Reserves inputs that have already been declared.
  void ReserveInputs(Context<T>* context) const {
    context->SetNumInputPorts(this->get_input_ports().size());
  }

  /// By default, allocates no state. Child classes that need state should
  /// override.
  virtual void ReserveState(Context<T>* context) const {}

  // SystemInterface objects are neither copyable nor moveable.
  explicit LeafSystem(const System<T>& other) = delete;
  LeafSystem& operator=(const System<T>& other) = delete;
  explicit LeafSystem(System<T>&& other) = delete;
  LeafSystem& operator=(System<T>&& other) = delete;
};

}  // namespace systems
}  // namespace drake
