#include "drake/systems/framework/output_port.h"

#include <memory>
#include <sstream>
#include <typeinfo>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template <typename T>
std::unique_ptr<AbstractValue> OutputPort<T>::Allocate(
    const Context<T>& context) const {
  DRAKE_ASSERT_VOID(get_system().CheckValidContext(context));
  return DoAllocate(context);
}

template <typename T>
void OutputPort<T>::Calc(const Context<T>& context,
                         AbstractValue* value) const {
  DRAKE_DEMAND(value != nullptr);
  DRAKE_ASSERT_VOID(CheckValidOutputType(context, *value));
  DRAKE_ASSERT_VOID(get_system().CheckValidContext(context));

  return DoCalc(context, value);
}

template <typename T>
const AbstractValue& OutputPort<T>::Eval(const Context<T>& context) const {
  DRAKE_ASSERT_VOID(get_system().CheckValidContext(context));
  return DoEval(context);
}

template <typename T>
OutputPort<T>::OutputPort(const System<T>& system, PortDataType data_type,
                          int size)
    : system_(system),
      index_(system.get_num_output_ports()),
      data_type_(data_type),
      size_(size) {
  if (size_ == kAutoSize) {
    DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
  }
}

template <typename T>
std::string OutputPort<T>::GetPortIdMsg() const {
  std::ostringstream oss;
  oss << "output port " << this->get_index() << " of "
      << NiceTypeName::Get(this->get_system()) << " System "
      << this->get_system().GetPath();
  return oss.str();
}

template <typename T>
void OutputPort<T>::CheckValidOutputType(const Context<T>& context,
                                         const AbstractValue& proposed) const {
  auto good = DoAllocate(context);  // Expensive!
  // Attempt to interpret these as BasicVectors.
  auto proposed_vec = dynamic_cast<const Value<BasicVector<T>>*>(&proposed);
  auto good_vec = dynamic_cast<const Value<BasicVector<T>>*>(good.get());
  if (proposed_vec && good_vec) {
    CheckValidBasicVector(context, good_vec->get_value(),
                          proposed_vec->get_value());
  } else {
    // At least one is not a BasicVector.
    CheckValidAbstractValue(context, *good, proposed);
  }
}

template <typename T>
void OutputPort<T>::CheckValidAbstractValue(
    const AbstractValue& good, const AbstractValue& proposed) const {
  if (typeid(proposed) != typeid(good)) {
    std::ostringstream oss;
    oss << "Calc(): expected AbstractValue output type "
        << NiceTypeName::Get(good) << " but got " << NiceTypeName::Get(proposed)
        << " for " << GetPortIdMsg();
    throw std::logic_error(oss.str());
  }
}

template <typename T>
void OutputPort<T>::CheckValidBasicVector(
    const BasicVector<T>& good, const BasicVector<T>& proposed) const {
  if (typeid(proposed) != typeid(good)) {
    std::ostringstream oss;
    oss << "Calc(): expected BasicVector output type "
        << NiceTypeName::Get(good) << " but got " << NiceTypeName::Get(proposed)
        << " for " << GetPortIdMsg();
    throw std::logic_error(oss.str());
  }
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
