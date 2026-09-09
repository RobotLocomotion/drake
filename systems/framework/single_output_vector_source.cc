#include "drake/systems/framework/single_output_vector_source.h"

namespace drake {
namespace systems {

template <typename T>
SingleOutputVectorSource<T>::~SingleOutputVectorSource() = default;

template <typename T>
SingleOutputVectorSource<T>::SingleOutputVectorSource(
    SystemScalarConverter converter, const BasicVector<T>& model_vector)
    : LeafSystem<T>(std::move(converter)) {
  this->DeclareVectorOutputPort(kUseDefaultName, model_vector,
                                &SingleOutputVectorSource<T>::CalcVectorOutput);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SingleOutputVectorSource);
