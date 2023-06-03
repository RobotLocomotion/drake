#include "drake/multibody/plant/rigid_transform_selector.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {

template <typename T>
RigidTransformSelector<T>::RigidTransformSelector(int index)
    : systems::LeafSystem<T>(systems::SystemTypeTag<RigidTransformSelector>{}),
      index_(index) {
  DRAKE_DEMAND(index >= 0);
  this->DeclareAbstractInputPort("vector", Value<ListType>());
  this->DeclareAbstractOutputPort("element",
                                  &RigidTransformSelector<T>::CalcOutput);
}

template <typename T>
template <typename U>
RigidTransformSelector<T>::
RigidTransformSelector(
    const RigidTransformSelector<U>& other)
    : RigidTransformSelector(other.index()) { }

template <typename T>
void RigidTransformSelector<T>::CalcOutput(
    const systems::Context<T>& context,
    ValueType* output) const {
  const ListType& vector =
      this->get_input_port().template Eval<ListType>(context);
  if (index_ >= static_cast<int>(vector.size())) {
    throw std::runtime_error(fmt::format(
        "RigidTransformSelector cannot select index {} from a vector of size "
        "{}.",
        index_, vector.size()));
  }
  *output = vector[index_];
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidTransformSelector)
