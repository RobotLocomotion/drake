#include "drake/systems/framework/leaf_output_port.h"

#include <memory>
#include <sstream>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace systems {

template <typename T>
void LeafOutputPort<T>::set_calculation_function(
    CalcVectorCallback vector_calc_function) {
  if (!vector_calc_function) {
    calc_function_ = nullptr;
  } else {
    // Wrap the vector-writing function with an AbstractValue-writing
    // function.
    calc_function_ = [this, vector_calc_function](const Context<T>& context,
                                                  AbstractValue* abstract) {
      // The abstract value must be a Value<BasicVector<T>>.
      auto value = dynamic_cast<Value<BasicVector<T>>*>(abstract);
      if (value == nullptr) {
        std::ostringstream oss;
        oss << "LeafOutputPort::Calc(): Expected a vector output type for "
            << this->GetPortIdString() << " but got a "
            << NiceTypeName::Get(*abstract) << " instead.";
        throw std::logic_error(oss.str());
      }
      vector_calc_function(context, &value->get_mutable_value());
    };
  }
}

template <typename T>
std::unique_ptr<AbstractValue> LeafOutputPort<T>::DoAllocate(
    const Context<T>& context) const {
  std::unique_ptr<AbstractValue> result;

  // Use the allocation function if available, otherwise clone the model
  // value.
  if (alloc_function_) {
    result = alloc_function_(context);
  } else {
    throw std::logic_error(
        "LeafOutputPort::DoAllocate(): " + this->GetPortIdString() +
        " has no allocation function so cannot be allocated.");
  }
  if (result.get() == nullptr) {
    throw std::logic_error(
        "LeafOutputPort::DoAllocate(): allocator returned a nullptr for " +
            this->GetPortIdString());
  }
  return result;
}

template <typename T>
void LeafOutputPort<T>::DoCalc(const Context<T>& context,
                               AbstractValue* value) const {
  if (calc_function_) {
    calc_function_(context, value);
  } else {
    throw std::logic_error("LeafOutputPort::DoCalc(): " +
                           this->GetPortIdString() +
                           " had no calculation function available.");
  }
}

template <typename T>
const AbstractValue& LeafOutputPort<T>::DoEval(
    const Context<T>& context) const {
  if (eval_function_) return eval_function_(context);
  // TODO(sherm1) Provide proper default behavior for an output port with
  // its own cache entry.
  DRAKE_ABORT_MSG("LeafOutputPort::DoEval(): NOT IMPLEMENTED YET");
  return *reinterpret_cast<const AbstractValue*>(0);
}

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafOutputPort)
