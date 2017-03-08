#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace systems {

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value)
    : SingleOutputVectorSource<T>(source_value.rows()),
      source_value_(source_value) {}

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(const T& source_value)
    : ConstantVectorSource(Vector1<T>::Constant(source_value)) {}

template <typename T>
ConstantVectorSource<T>::~ConstantVectorSource() = default;

template <typename T>
void ConstantVectorSource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  *output = source_value_;
}

// Explicitly instantiates on the most common scalar types.
template class ConstantVectorSource<double>;
template class ConstantVectorSource<AutoDiffXd>;
template class ConstantVectorSource<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
