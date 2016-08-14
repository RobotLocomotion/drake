#include "drake/systems/framework/primitives/constant_source.h"

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value) :
    source_value_(source_value) {
  // TODO(amcastro-tri): Add output ports using System<T>::declare_output_port
  // after #3102 is merged.
}

template <typename T>
std::unique_ptr<ContextBase<T>>
ConstantVectorSource<T>::CreateDefaultContext() const {
  std::unique_ptr<Context<T>> context(new Context<T>);
  // TODO(amcastro-tri): Remove this method after #3102 since it will provide a
  // default implementation.

  // A ConstantVectorSource has no input ports.
  context->SetNumInputPorts(0);
  return std::unique_ptr<ContextBase<T>>(context.release());
}

template <typename T>
std::unique_ptr<SystemOutput<T>> ConstantVectorSource<T>::AllocateOutput(
    const ContextBase<T>& context) const {
  // TODO(amcastro-tri): Remove this method after #3102 since it will provide a
  // default implementation.

  // A ConstantVectorSource has just one output port, a BasicVector of the size
  // specified at construction time.
  std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
  {
    std::unique_ptr<BasicVector<T>> data(
        new BasicVector<T>(source_value_.rows()));
    std::unique_ptr<OutputPort<T>> port(new OutputPort<T>(std::move(data)));
    output->get_mutable_ports()->push_back(std::move(port));
  }
  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
void ConstantVectorSource<T>::EvalOutput(const ContextBase<T>& context,
                                   SystemOutput<T>* output) const {
  // Checks on the output structure are assertions, not exceptions, since
  // failures would reflect a bug in the ConstantVectorSource implementation,
  // not user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.

  // Asserts that there is only one output port.
  DRAKE_ASSERT(output->get_num_ports() == 1);
  // TODO(amcastro-tri): change to:
  // auto& output_vector = this->get_output_vector(context, 0);
  // where output_vector will be an Eigen expression.
  VectorInterface<T>* output_vector =
      output->get_mutable_port(0)->GetMutableVectorData();
  DRAKE_ASSERT(output_vector != nullptr);
  DRAKE_ASSERT(output_vector->get_value().rows() == source_value_.rows());

  // TODO(amcastro-tri): System<T> should provide interfaces to directly get the
  // actually useful Eigen vectors like so:
  // auto input_vector = System<T>::get_input_port(0).get_vector(context);
  // auto output_vector =
  //   System<T>::get_output_port(0).get_mutable_vector(context);
  // or the alternative:
  // auto& output_vector = this->get_mutable_output_vector(context, 0);
  output_vector->get_mutable_value() = source_value_;
}

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT ConstantVectorSource<double>;

}  // namespace systems
}  // namespace drake
