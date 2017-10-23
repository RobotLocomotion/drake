#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/default_scalars.h"

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
BasicVector<T>& ConstantVectorSource<T>::get_mutable_source_value(
    Context<T>* context) {
  return this->GetMutableNumericParameter(context, source_value_index_);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ConstantVectorSource)
