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

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.

template class OutputPort<double>;
template class OutputPort<AutoDiffXd>;
template class OutputPort<symbolic::Expression>;
template class OutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class OutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake
