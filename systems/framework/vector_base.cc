#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

template <typename T>
VectorBase<T>::~VectorBase() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorBase);
