#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value)
    : ConstantVectorSource(BasicVector<T>(source_value)) {}

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const BasicVector<T>& source_value)
    : SingleOutputVectorSource<T>(source_value) {
  source_value_index_ = this->DeclareNumericParameter(source_value);
}

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(const T& source_value)
    : ConstantVectorSource(Vector1<T>::Constant(source_value)) {}

template <typename T>
ConstantVectorSource<T>::~ConstantVectorSource() = default;

template <typename T>
void ConstantVectorSource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  *output = get_source_value(context).get_value();
}

template <typename T>
const BasicVector<T>& ConstantVectorSource<T>::get_source_value(
    const Context<T>& context) const {
  return this->GetNumericParameter(context, source_value_index_);
}

template <typename T>
BasicVector<T>* ConstantVectorSource<T>::get_mutable_source_value(
    Context<T>* context) {
  return this->GetMutableNumericParameter(context, source_value_index_);
}

// Explicitly instantiates on the most common scalar types.
template class ConstantVectorSource<double>;
template class ConstantVectorSource<AutoDiffXd>;
template class ConstantVectorSource<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
