#include "drake/systems/framework/vector_base.h"

#include "drake/common/fmt_eigen.h"

namespace drake {
namespace systems {

template <typename T>
VectorBase<T>::~VectorBase() {}

template <typename T>
std::string VectorBase<T>::to_string() const {
  return fmt::to_string(fmt_eigen(CopyToVector().transpose()));
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorBase);
