#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
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

  // =========================================================================
  // Implementations of System<T> methods.

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    std::unique_ptr<LeafContext<T>> context(new LeafContext<T>);
    // Reserve inputs that have already been declared.
    context->SetNumInputPorts(this->get_num_input_ports());
    // Reserve continuous state via delegation to subclass.
    context->set_continuous_state(this->AllocateContinuousState());
    // Reserve discrete state via delegation to subclass.
    ReserveDiscreteState(context.get());
    return std::unique_ptr<Context<T>>(context.release());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
    for (const auto& descriptor : this->get_output_ports()) {
      output->get_mutable_ports()->emplace_back(
          new OutputPort(AllocateOutputVector(descriptor)));
    }
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  /// Returns the AllocateContinuousState value, which may be nullptr.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return AllocateContinuousState();
  }

 protected:
  LeafSystem() {}

  // =========================================================================
  // New methods for subclasses to override

  /// Returns a ContinuousState used to implement both CreateDefaultContext and
  /// AllocateTimeDerivatives. By default, allocates no state. Systems with
  /// continuous state variables should override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const {
    return nullptr;
  }

  /// Reserves the discrete state as required by CreateDefaultContext.  By
  /// default, reserves no state. Systems with discrete state should override.
  virtual void ReserveDiscreteState(LeafContext<T>* context) const {}

  /// Given a port descriptor, allocate the vector storage.  The default
  /// implementation in this class allocates a BasicVector.  Subclasses can
  /// override to use output vector types other than BasicVector.  The
  /// descriptor must match a port declared via DeclareOutputPort.
  virtual std::unique_ptr<BasicVector<T>> AllocateOutputVector(
      const SystemPortDescriptor<T>& descriptor) const {
    return std::make_unique<BasicVector<T>>(descriptor.get_size());
  }

 private:
  // SystemInterface objects are neither copyable nor moveable.
  explicit LeafSystem(const System<T>& other) = delete;
  LeafSystem& operator=(const System<T>& other) = delete;
  explicit LeafSystem(System<T>&& other) = delete;
  LeafSystem& operator=(System<T>&& other) = delete;
};

}  // namespace systems
}  // namespace drake
