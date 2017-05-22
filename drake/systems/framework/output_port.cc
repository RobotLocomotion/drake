#include "drake/systems/framework/output_port.h"

#include <sstream>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

//==============================================================================
//                                OUTPUT PORT
//==============================================================================
template <typename T>
std::unique_ptr<AbstractValue> OutputPort<T>::Allocate(
    const Context<T>* context) const {
  if (context != nullptr) {
    DRAKE_ASSERT_VOID(get_system()->CheckValidContext(*context));
  }
  return DoAllocate(context);
}

// Note that this is just sugar. It calls the real Allocate() method above,
// assumes the AbstractValue contains a BasicVector, digs out that vector and
// returns it to the caller, getting rid of the rest of the AbstractValue
// infrastructure.
// TODO(sherm1) Consider whether to nuke this method altogether.
template <typename T>
std::unique_ptr<BasicVector<T>> OutputPort<T>::AllocateVector(
    const Context<T>* context) const {
  std::unique_ptr<AbstractValue> abstract = Allocate(context);
  // The abstract value must be a VectorValue<T>.
  VectorValue<T>* value = dynamic_cast<VectorValue<T>*>(abstract.get());
  if (value == nullptr) {
    std::ostringstream oss;
    oss << "OutputPort::AllocateVector(): Expected a vector type from the "
           "allocator for "
        << this->GetPortIdMsg() << " but got a " << NiceTypeName::Get(*abstract)
        << " instead.";
    throw std::logic_error(oss.str());
  }

  return value->release_vector();
  // The empty AbstractValue shell is destructed here.
}

template <typename T>
void OutputPort<T>::Calc(const Context<T>& context,
                         AbstractValue* value) const {
  DRAKE_ASSERT_VOID(get_system()->CheckValidContext(context));
  DRAKE_DEMAND(value != nullptr);
  // TODO(sherm1) Validate abstract value object.
  return DoCalc(context, value);
}

template <typename T>
const AbstractValue& OutputPort<T>::Eval(const Context<T>& context) const {
  DRAKE_ASSERT_VOID(get_system()->CheckValidContext(context));
  return DoEval(context);
}

template <typename T>
std::string OutputPort<T>::GetPortIdMsg() const {
  std::ostringstream oss;
  oss << "output port " << this->get_index() << " of "
      << NiceTypeName::Get(*this->get_system()) << " System "
      << this->get_system()->GetPath();
  return oss.str();
}

//==============================================================================
//                            LEAF OUTPUT PORT
//==============================================================================

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
      // The abstract value must be a VectorValue<T>.
      auto value = dynamic_cast<Value<BasicVector<T>*>*>(abstract);
      if (value == nullptr) {
        std::ostringstream oss;
        oss << "OutputPort::Calc(): Expected a vector output type for "
            << this->GetPortIdMsg() << " but got a "
            << NiceTypeName::Get(*abstract) << " instead.";
        throw std::logic_error(oss.str());
      }
      vector_calc_function(context, value->get_mutable_value());
    };
  }
}

template <typename T>
std::unique_ptr<AbstractValue> LeafOutputPort<T>::DoAllocate(
    const Context<T>* context) const {
  std::unique_ptr<AbstractValue> result;

  // Use the allocation function if available, otherwise clone the model
  // value.
  if (alloc_function_) {
    result = alloc_function_(context);
  } else if (model_value_) {
    result = model_value_->Clone();
  } else {
    std::ostringstream oss;
    oss << "LeafOutputPort::DoAllocate(): " << this->GetPortIdMsg()
        << " has neither an allocation function nor a model value so cannot"
            " be allocated.";
    throw std::logic_error(oss.str());
  }
  DRAKE_DEMAND(result.get() != nullptr);
  return result;
}

template <typename T>
void LeafOutputPort<T>::DoCalc(const Context<T>& context,
                               AbstractValue* value) const {
  if (calc_function_) {
    calc_function_(context, value);
  } else {
    std::ostringstream oss;
    oss << "LeafOutputPort::DoCalc(): " << this->GetPortIdMsg()
        << " had no calculation function available.";
    throw std::logic_error(oss.str());
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

template class OutputPort<double>;
template class OutputPort<AutoDiffXd>;
template class OutputPort<symbolic::Expression>;
template class OutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class OutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

template class LeafOutputPort<double>;
template class LeafOutputPort<AutoDiffXd>;
template class LeafOutputPort<symbolic::Expression>;
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake
