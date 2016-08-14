#include "drake/systems/framework/primitives/integrator.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
Integrator<T>::Integrator(int length)
    : length_(length) {}

template <typename T>
std::unique_ptr<ContextBase<T>> Integrator<T>::CreateDefaultContext() const {
  std::unique_ptr<Context<T>> context(new Context<T>);
  context->SetNumInputPorts(1);
  // The integrator has a state vector of the same dimension as its input
  // and output.
  context->get_mutable_state()->continuous_state.reset(new ContinuousState<T>(
      std::make_unique<BasicStateVector<T>>(length_), 0 /* size of q */,
      0 /* size of v */, length_ /* size of z */));
  return std::unique_ptr<ContextBase<T>>(context.release());
}

template <typename T>
std::unique_ptr<SystemOutput<T>> Integrator<T>::AllocateOutput(
    const ContextBase<T>& context) const {
  // An Integrator has just one output port, a BasicVector of the size specified
  // at construction time.
  std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
  {
    output->get_mutable_ports()->push_back(std::make_unique<OutputPort<T>>(
        std::make_unique<BasicVector<T>>(length_)));
  }
  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
std::unique_ptr<ContinuousState<T>> Integrator<T>::AllocateTimeDerivatives()
    const {
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicStateVector<T>>(length_), 0 /* size of q */,
      0 /* size of v */, length_ /* size of z */);
}

template <typename T>
void Integrator<T>::EvalOutput(const ContextBase<T>& context,
                               SystemOutput<T>* output) const {
  DRAKE_ASSERT(output->get_num_ports() == 1);
  VectorInterface<T>* output_port =
      output->get_mutable_port(0)->GetMutableVectorData();
  DRAKE_ASSERT(output_port != nullptr);
  DRAKE_ASSERT(output_port->get_value().rows() == length_);

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_port->get_mutable_value() =
      context.get_state().continuous_state->get_state().CopyToVector();
}

template <typename T>
void Integrator<T>::EvalTimeDerivatives(const ContextBase<T>& context,
                                        ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT(context.get_num_input_ports() == 1);
  const VectorInterface<T>* input = context.get_vector_input(0);
  derivatives->get_mutable_state()->SetFromVector(input->get_value());
}

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT Integrator<double>;

}  // namespace systems
}  // namespace drake
