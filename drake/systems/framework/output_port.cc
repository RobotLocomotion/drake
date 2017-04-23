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
           "allocator for output port "
        << this->get_index() << " of System " << this->get_system()->GetPath()
        << " but got a " << NiceTypeName::Get(*abstract) << " instead.";
    throw std::logic_error(oss.str());
  }

  return value->ExtractBasicVector();
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

//==============================================================================
//                            LEAF OUTPUT PORT
//==============================================================================
template <typename T>
std::unique_ptr<AbstractValue> LeafOutputPort<T>::DoAllocate(
    const Context<T>* context) const {
  std::unique_ptr<AbstractValue> result;

  // Use the allocation function if available, otherwise clone the model
  // value.
  if (alloc_function_) {
    result = alloc_function_(context);
  } else if (model_value_ != nullptr) {
    result = model_value_->Clone();
  } else {
    std::ostringstream oss;
    oss << "LeafOutputPort::DoAllocate(): Output port " << this->get_index()
        << " of System " << NiceTypeName::Get(*this->get_system())
        << " has neither an allocation function nor a model value so cannot"
           " be allocated.";
    throw std::logic_error(oss.str());
  }
  DRAKE_DEMAND(result.get() != nullptr);
  return result;
}

// TODO(sherm1) Call CalcOutput for backwards compatibility.
template <typename T>
void LeafOutputPort<T>::DoCalc(const Context<T>& context,
                               AbstractValue* value) const {
  if (calc_function_) {
    calc_function_(context, value);
  } else {
    std::ostringstream oss;
    oss << "LeafOutputPort::DoCalc(): Output port " << this->get_index()
        << " had no calculation function available.";
    throw std::logic_error(oss.str());
  }
}

template <typename T>
const AbstractValue& LeafOutputPort<T>::DoEval(
    const Context<T>& context) const {
  if (eval_function_)
    return eval_function_(context);
  // TODO(sherm1) Provide proper default behavior for an output port with
  // its own cache entry.
  DRAKE_ABORT_MSG("LeafOutputPort::DoEval: NOT IMPLEMENTED YET");
  return *reinterpret_cast<const AbstractValue*>(0);
}

template class OutputPort<double>;
template class LeafOutputPort<double>;
template class DiagramOutputPort<double>;

template class OutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class DiagramOutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;

template class OutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;
template class DiagramOutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

template class OutputPort<AutoDiffXd>;
template class LeafOutputPort<AutoDiffXd>;
template class DiagramOutputPort<AutoDiffXd>;

template class OutputPort<symbolic::Expression>;
template class LeafOutputPort<symbolic::Expression>;
template class DiagramOutputPort<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
